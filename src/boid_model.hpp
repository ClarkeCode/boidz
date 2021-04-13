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

    namespace detail {
        using linalgvec2 = linalg::aliases::float2;
        const float _pi = PI;
        //Conversion function to turn linalg vector into a raylib vector
        Vector2 compat(linalgvec2 const& vect) { return Vector2{vect.x, vect.y}; }
        //Produces linalg vector from a radian angle
        float2 produceUnitVector(float radianAngle) { return float2(cosf(radianAngle), sinf(radianAngle)); }
        //Produce a linalg position vector where vector is {0-maxX, 0-maxY}
        float2 produceRandomPos(randutil::RandomNumberFactory<>& randomer, float maxX, float maxY) {
            return float2(randomer.produceRandom<float>(0.0f, maxX), randomer.produceRandom<float>(0.0f, maxY));
        }
        //Normalize function safe for zero-length vectors
        float2 snormalize(float2 vect) {
            if (linalg::length(vect) != 0.0f)
                return linalg::normalize(vect);
            return vect;
        }
        //Ensures a given vector is not longer than the provided maximum length
        float2 vclamp(float2 vect, float maxVectLength) {
            if (linalg::length(vect) > maxVectLength)
                return maxVectLength * snormalize(vect);
            return vect;
        }

        //Conversion function to convert a degree angle to a radian scalar
        template<typename T = float> T degreeToRadian(T value) { return value * _pi / ((T) 180.0f); }
        //Conversion function to convert a radian scalar to a degree angle
        template<typename T = float> T radianToDegree(T value) { return value * ((T) 180.0f) / _pi; }
    }
    
    class SpacialComponent : public ecs::Component {
        public:
        float2 pos, vel, acc;
        float mass, maxSpeed, maxForce;
        float visionConeDegrees;
        
        SpacialComponent(float2 position, float2 velocity, float2 acceleration, float componentMass, float max_speed, float max_force, float vision_cone_degree) : 
            pos(position), vel(velocity), acc(acceleration), mass(componentMass), maxSpeed(max_speed), maxForce(max_force), visionConeDegrees(vision_cone_degree) {}
        SpacialComponent() : SpacialComponent(float2(0,0), float2(0,0), float2(0,0), 1.0f, 100.0f, 100.0f, 270.0f) {}
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
        inline bool isWithinVision(Boid::ptr_t otherBoid) const {
            float2 betweenVec = otherBoid->spacialInfo.pos - this->spacialInfo.pos;
            float ang = angle(this->getVel(), betweenVec);
            return !(   ang > (detail::degreeToRadian(this->spacialInfo.visionConeDegrees/2)) && 
                        ang < (2*3.1415f - detail::degreeToRadian(this->spacialInfo.visionConeDegrees/2)));
        }
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

        //Gives Boids a force to stay away from neighbouring boids
        class SeparatorForce : public Force {
            public:
            float desiredSeparation;
            SeparatorForce(float separationRadius, float desiredWeight = 1.0f) : Force(desiredWeight), desiredSeparation(separationRadius) {}
            virtual float2 produceSteeringVector(Boid::Flock const& flock, Boid::ptr_t const& actor) const override {
                float2 steeringVector(0.0f);
                float2 desiredVelocity(0.0f);

                for(Boid::ptr_t const& otherBoid : flock) {
                    //Neighbour test
                    float dist = distance(actor->getPos(), otherBoid->getPos());
                    if (dist > 0.0f && dist < desiredSeparation && actor->isWithinVision(otherBoid)) {
                        //renolds 1999 separation behaviour: favour closer neighbours more heavily
                        desiredVelocity += linalg::normalize(actor->getPos() - otherBoid->getPos()) / dist;
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

        class CohesionForce : public Force {
            public:
            float desiredCohesionRadius;
            CohesionForce(float cohesionRadius, float desiredWeight = 1.0f) : Force(desiredWeight), desiredCohesionRadius(cohesionRadius) {}
            virtual float2 produceSteeringVector(Boid::Flock const& flock, Boid::ptr_t const& actor) const override {
                float2 targetPosition(0.0f);
                int numNeighbourBoid = 0;
                for(Boid::ptr_t const& otherBoid : flock) {
                    //Neighbour test
                    float dist = distance(actor->getPos(), otherBoid->getPos());
                    if (dist > 0.0f && dist < desiredCohesionRadius && actor->isWithinVision(otherBoid)) {
                        targetPosition += otherBoid->getPos();
                        numNeighbourBoid++;
                    }
                }
                if (numNeighbourBoid == 0)
                    return float2(0.0f);
                
                float2 desiredVelocity = targetPosition - actor->getPos();
                float2 steeringVector = desiredVelocity - actor->getVel();

                return steeringVector;
            }
        };

        class AlignmentForce : public Force {
            public:
            float alignmentEffectRadius;
            AlignmentForce(float alignmentRadius, float desiredWeight = 1.0f) : Force(desiredWeight), alignmentEffectRadius(alignmentRadius) {};
            virtual float2 produceSteeringVector(Boid::Flock const& flock, Boid::ptr_t const& actor) const override {
                float2 averageVelocity(0.0f);
                int numNeighbourBoid = 0;
                for(Boid::ptr_t const& otherBoid : flock) {
                    //Neighbour test
                    float dist = distance(actor->getPos(), otherBoid->getPos());
                    if (dist > 0.0f && dist < alignmentEffectRadius && actor->isWithinVision(otherBoid)) {
                        averageVelocity += otherBoid->getVel();
                        numNeighbourBoid++;
                    }
                }
                if (numNeighbourBoid == 0)
                    return float2(0.0f);
                
                float2 desiredVelocity = averageVelocity / numNeighbourBoid;
                float2 steeringVector = desiredVelocity - actor->getVel();

                return steeringVector;
            }
        };

        //Gives Boids a force to continue in the direction they were going
        class AcceleratorForce : public Force {
            public:
            AcceleratorForce(float desiredWeight = 0.1f) : Force(desiredWeight) {}
            virtual float2 produceSteeringVector(Boid::Flock const&, Boid::ptr_t const& actor) const override {
                float2 steeringVector;
                steeringVector = actor->getVel();
                if (length(steeringVector) < 20.0f)
                    steeringVector = (detail::snormalize(steeringVector) * 20.0f);
                return steeringVector;
            }
        };

        /*
        TODO: add a distinction between forces to allow for a force to only apply to an individual Boid
        ie Force -> IndividualForce -> SpecializedIndividualForce (SeekMouse?)
        Have SIVs take their target information as non-owning pointers on construction, implement produceSteeringVector method by ignoring the parameters
        */

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



    class MovementSystem : public ecs::System {
        public:
        bool positionLimitWrapping;
        float2 limitsXY;
        forces::ForceManager forceManager;
        MovementSystem() : positionLimitWrapping(true) {
            using namespace boid::forces;
            forceManager.attachForce<SeparatorForce>(50.0f);
            forceManager.attachForce<CohesionForce>(100.0f, 0.5f);
            forceManager.attachForce<AlignmentForce>(100.0f, 0.5f);
            forceManager.attachForce<AcceleratorForce>(0.3f);
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
                boidp->spacialInfo.acc = detail::vclamp(boidp->getAcc(), boidp->spacialInfo.maxForce);
                boidp->spacialInfo.vel += frametime * boidp->spacialInfo.acc;
                boidp->spacialInfo.vel = detail::vclamp(boidp->getVel(), boidp->spacialInfo.maxSpeed);
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

                    DrawLineEx(compat(boidp->spacialInfo.pos), compat(
                        boidp->spacialInfo.pos + linalg::rot(detail::degreeToRadian(boidp->spacialInfo.visionConeDegrees/2), boidp->spacialInfo.vel)), 1, ORANGE);
                    DrawLineEx(compat(boidp->spacialInfo.pos), compat(
                        boidp->spacialInfo.pos + linalg::rot(2*3.1415f - detail::degreeToRadian(boidp->spacialInfo.visionConeDegrees/2), boidp->spacialInfo.vel)), 1, PURPLE);

                    for (Boid::ptr_t const& other : flock) {
                        if (boidp->isWithinVision(other) && distance(boidp->getPos(), other->getPos()) < 100 ) {
                            DrawLineEx(compat(boidp->getPos()), compat(other->getPos()), 1, GRAY);
                        }
                    }
                    // DrawCircleSectorLines(compat(boidp->spacialInfo.pos), 100.0f, 0.0f, boidp->spacialInfo.visionConeDegrees, 8, BLACK);
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