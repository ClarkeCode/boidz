#ifndef GAME_MODEL_BOID
#define GAME_MODEL_BOID
#include <vector>
#include "lib/ecs.hpp"
#include "lib/linalg.h"

//TODO: Following includes should be moved into cpp implementation file(s)
#include "lib/random_manager.hpp"   //RandomNumberFactory
#include "raylib.h"                 //renderingsystem
#include <utility>                  //move
#include <algorithm>                //for_each
#include <cmath>                    //sinf cosf etc

namespace boid {
    using namespace linalg::aliases;
    
    class SpacialComponent : public ecs::Component {
        public:
        float2 pos, vel, acc;
        float mass, maxSpeed, maxForce;
        
        SpacialComponent(float2 position, float2 velocity, float2 acceleration, float componentMass, float max_speed, float max_force) : 
            pos(position), vel(velocity), acc(acceleration), mass(componentMass), maxSpeed(max_speed), maxForce(max_force) {}
        SpacialComponent() : SpacialComponent(float2(0,0), float2(0,0), float2(0,0), 1.0f, 100.0f, 10.0f) {}
    };

    class VisualComponent : public ecs::Component {
        public:
        int debugLevel;
        bool isDisplayable, showDebugInfo;

        VisualComponent() : debugLevel(0), isDisplayable(true), showDebugInfo(false) {}
    };

    class Boid { // : public ecs::Entity {
        public:
        using ptr_t = std::shared_ptr<Boid>;
        using Flock = std::vector<Boid::ptr_t>;

        SpacialComponent spacialInfo;
        VisualComponent visualInfo;

        Boid() = default;
        inline void setPosition(float2 xy) { spacialInfo.pos = xy; }
        inline float getPosX() const { return spacialInfo.pos.x; }
        inline float getPosY() const { return spacialInfo.pos.y; }
        inline float2 getPos() const { return spacialInfo.pos; }
        inline float2 getVel() const { return spacialInfo.vel; }
        inline float2 getAcc() const { return spacialInfo.acc; }
    };

    namespace forces {
        class Force {
            public:
            using ptr_t = std::unique_ptr<Force>;
            float weight;
            virtual float2 produceSteeringVector(Boid::Flock const& flock, Boid::ptr_t const& actor) const = 0;
            Force(float chosenWeight = 1.0f) : weight(chosenWeight) {}
            virtual ~Force() = default;
        };

        class SeparatorForce : public Force {
            public:
            float desiredSeparation;
            SeparatorForce(float separationRadius, float desiredWeight = 1.0f) : Force(desiredWeight), desiredSeparation(separationRadius) {}
            virtual float2 produceSteeringVector(Boid::Flock const& flock, Boid::ptr_t const& actor) const override {
                float2 steeringVector(0.0f);
                linalg::aliases::float2 desiredVelocity(0.0f);

                for(Boid::ptr_t const& otherBoid : flock) {
                    float dist = distance(actor->getPos(), otherBoid->getPos());
                    if (dist > 0.0f && dist < desiredSeparation) {
                        //renolds 1999 separation behaviour: favour closer neighbours more heavily
                        desiredVelocity += linalg::normalize(actor->getPos() - otherBoid->getPos()) / dist * dist;
                    }
                }

                if (linalg::length(desiredVelocity) != 0.0f) {
                    //Scale desired velocity to be at maximum speed of the boid
                    desiredVelocity = actor->spacialInfo.maxSpeed * linalg::normalize(desiredVelocity);
                    steeringVector = desiredVelocity - actor->spacialInfo.vel;// = desired_velocity - current_velocity
                }
                return steeringVector;
            }
        };

        class ForceManager {
            public:
            std::vector<Force::ptr_t> attachedForces;
            ForceManager() = default;

            //Applies all attached forces to the provided boid flock
            //Note: Function is O(n^3) due to calling Force::produceSteeringVector function
            inline void applyForces(Boid::Flock const& flock) const {
                //std::for_each(flock.begin(), flock.end(), [](Boid::ptr_t const& currentBoid)->void{currentBoid->spacialInfo.acc = float2(0.0f);});
                std::for_each(flock.begin(), flock.end(), [this, &flock](Boid::ptr_t const& currentBoid)->void{
                    currentBoid->spacialInfo.acc = float2(0.0f);
                    std::for_each(attachedForces.begin(), attachedForces.end(), [this, &flock, &currentBoid](Force::ptr_t const& currentForce)->void{
                        currentBoid->spacialInfo.acc += currentForce->weight * currentForce->produceSteeringVector(flock, currentBoid);
                    });
                });
            }
            
            //Attach a new instance of a specified force with any number of constructor arguments
            template<typename SpecializedForce, typename ... ConstructorArgumentTypes>
            void attachForce(ConstructorArgumentTypes... constructorArguments) {
                attachedForces.emplace_back(std::make_unique<SpecializedForce>(constructorArguments...));
            }

            //Attach a new instance of a specified force with default construction
            template<typename SpecializedForce>
            void attachForce() {
                attachedForces.emplace_back(std::make_unique<SpecializedForce>());
            }
        };
    }

    namespace detail {
        using linalgvec2 = linalg::aliases::float2;
        //Conversion function to turn linalg vector into a raylib vector
        Vector2 compat(linalgvec2 const& vect) { return Vector2{vect.x, vect.y}; }
        //Produces linalg vector from a radian angle
        float2 produceUnitVector(float radianAngle) { return float2(cosf(radianAngle), sinf(radianAngle)); }
        //Produce a linalg position vector where vector is {0-maxX, 0-maxY}
        float2 produceRandomPos(randutil::RandomNumberFactory<>& randomer, float maxX, float maxY) {
            return float2(randomer.produceRandom<float>(0.0f, maxX), randomer.produceRandom<float>(0.0f, maxY));
        }
    }

    class MovementSystem : public ecs::System {
        public:
        bool positionLimitWrapping;
        float2 limitsXY;
        forces::ForceManager forceManager;
        MovementSystem() : positionLimitWrapping(true) {
            using namespace boid::forces;
            forceManager.attachForce<SeparatorForce>(50.0f);
        }
        //TODO: May need to keep screensize information as member attributes that are initialized at construction
        virtual void process(ecs::Entity::EntityContainer const& entities) override {
            //TODO: Implement movement mehaviour of boids with SpacialComponents, rules for alignment, separation, cohesion may need to be a separate system
        }

        inline void process(Boid::Flock const& flock, float frametime) {
            //Apply forces to each member of the flock
            forceManager.applyForces(flock);

            //Apply Euler-integration style movement
            std::for_each(flock.begin(), flock.end(), [this, frametime](Boid::ptr_t const& boidp)->void{
                boidp->spacialInfo.vel += frametime * boidp->spacialInfo.acc;
                boidp->spacialInfo.pos += frametime * boidp->spacialInfo.vel;
                if (positionLimitWrapping) {
                    if (boidp->getPosX() < 0) boidp->spacialInfo.pos.x = limitsXY.x;
                    if (boidp->getPosY() < 0) boidp->spacialInfo.pos.y = limitsXY.y;
                    if (boidp->getPosX() > limitsXY.x) boidp->spacialInfo.pos.x = 0.0f;
                    if (boidp->getPosY() > limitsXY.y) boidp->spacialInfo.pos.y = 0.0f;
                }
            });
        }
    };

    
    class RenderingSystem : public ecs::System {
        public:
        virtual void process(ecs::Entity::EntityContainer const& entities) override {
            //TODO: Implement drawing behaviours of boids with SpacialComponents and VisualComponents
        }

        inline void render(Boid::Flock const& flock) const {
            using namespace detail;
            for (Boid::ptr_t const& boidp : flock) {
                DrawCircleV(compat(boidp->spacialInfo.pos), 5, boidp->visualInfo.showDebugInfo ? RED : BLACK);
                DrawLineEx(compat(boidp->spacialInfo.pos), compat(boidp->spacialInfo.pos + (boidp->spacialInfo.vel)), 1, BLACK);

                if (boidp->visualInfo.showDebugInfo) {
                    DrawLineEx(compat(boidp->spacialInfo.pos), compat(boidp->spacialInfo.pos + (boidp->spacialInfo.acc)), 1, GREEN);
                }
            }
        }
    };

    class BoidModel {
        public:
        float2 modelDimensions;

        Boid::Flock flock;

        MovementSystem movementSystem;
        RenderingSystem renderingSystem;

        randutil::RandomNumberFactory<float> randomManager;

        BoidModel() = delete;
        BoidModel(float2 worldDimensions) : modelDimensions(worldDimensions) {
            randutil::RandomNumberFactory<> randomFactory;
            movementSystem.limitsXY = modelDimensions;
            
            for (int x = 0; x < 20; x++) { flock.push_back(std::move(std::make_shared<Boid>())); }

            std::for_each(flock.begin(), flock.end(), [this, randomFactory](Boid::ptr_t& boidp)mutable->void{
                boidp->setPosition(detail::produceRandomPos(randomFactory, modelDimensions.x, modelDimensions.y));
                boidp->spacialInfo.vel = boidp->spacialInfo.maxSpeed * detail::produceUnitVector(randomFactory.produceRandom<float>(0.0f, 2.0f*3.14159f));
            });

            flock[0]->spacialInfo.pos = float2(modelDimensions.x/2.0f, modelDimensions.y/2.0f);
            flock[0]->visualInfo.showDebugInfo = true;
        };

        inline void updateModel(float frametime) { movementSystem.process(flock, frametime); }
        inline void renderModel() const { renderingSystem.render(flock); }
    };
}

#endif