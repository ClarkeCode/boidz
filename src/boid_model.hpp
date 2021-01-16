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
    };

    namespace forces {
        class Force {
            public:
            using ptr_t = std::unique_ptr<Force>;
            virtual float2 produceSteeringVector(Boid::Flock const& flock, Boid::ptr_t const& actor) const = 0;
            virtual ~Force() = default;
        };

        class SeekForce : public Force {
            public:
            SeekForce() : Force() {}
            virtual float2 produceSteeringVector(Boid::Flock const& flock, Boid::ptr_t const& actor) const override {
                float2 target; //select target
                float2 desiredVelocity;// = linalg::normalize(position - target)
                float2 steeringVector;// = desired_velocity - velocity
                return steeringVector;
            }
        };

        class ForceManager {
            public:
            std::vector<Force::ptr_t> forces;
            ForceManager() = default;
            
            //Attach a new instance of a specified force with any number of constructor arguments
            template<typename SpecializedForce, typename ... ConstructorArgumentTypes>
            void attachForce(ConstructorArgumentTypes... constructorArguments) {
                forces.emplace_back(std::make_unique<SpecializedForce>(constructorArguments...));
            }

            //Attach a new instance of a specified force with default construction
            template<typename SpecializedForce>
            void attachForce() {
                forces.emplace_back(std::make_unique<SpecializedForce>());
            }
        };
    }

    namespace detail {
        using linalgvec2 = linalg::aliases::float2;
        //Conversion function to turn linalg vector into a raylib vector
        Vector2 compat(linalgvec2 const& vect) { return Vector2{vect.x, vect.y}; }
        //Produces linalg vector from a radian angle
        linalgvec2 produceUnitVector(float radianAngle) { return linalgvec2(cosf(radianAngle), sinf(radianAngle)); }
        //Produce a linalg position vector where vector is {0-maxX, 0-maxY}
        linalgvec2 produceRandomPos(randutil::RandomNumberFactory<>& randomer, float maxX, float maxY) { 
            return linalgvec2(randomer.produceRandom<float>(0.0f, maxX), randomer.produceRandom<float>(0.0f, maxY));
        }
    }

    class MovementSystem : public ecs::System {
        public:
        forces::ForceManager forceManager;
        MovementSystem() {
            using namespace boid::forces;
            forceManager.attachForce<SeekForce>();
        }
        //TODO: May need to keep screensize information as member attributes that are initialized at construction
        virtual void process(ecs::Entity::EntityContainer const& entities) override {
            //TODO: Implement movement mehaviour of boids with SpacialComponents, rules for alignment, separation, cohesion may need to be a separate system
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
                DrawLineEx(compat(boidp->spacialInfo.pos), compat(boidp->spacialInfo.pos + (30 * boidp->spacialInfo.vel)), 1, BLACK);
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
            
            for (int x = 0; x < 10; x++) { flock.push_back(std::move(std::make_shared<Boid>())); }

            std::for_each(flock.begin(), flock.end(), [this, randomFactory](Boid::ptr_t& boidp)mutable->void{
                boidp->setPosition(detail::produceRandomPos(randomFactory, modelDimensions.x, modelDimensions.y));
                boidp->spacialInfo.vel = boidp->spacialInfo.maxSpeed * detail::produceUnitVector(randomFactory.produceRandom<float>(0.0f, 2.0f*3.14159f));
            });

            flock[0]->spacialInfo.pos = float2(modelDimensions.x/2.0f, modelDimensions.y/2.0f);
            flock[0]->visualInfo.showDebugInfo = true;
        };

        inline void updateModel(float frametime) {}
        inline void renderModel() const { renderingSystem.render(flock); }
    };
}

#endif