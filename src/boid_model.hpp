#ifndef GAME_MODEL_BOID
#define GAME_MODEL_BOID
#include <vector>
#include "lib/ecs.hpp"
#include "lib/linalg.h"
#include <random> //randomization
#include <utility> //move

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
        using BoidFlock = std::vector<Boid::ptr_t>;

        SpacialComponent spacialInfo;
        VisualComponent visualInfo;

        Boid() = default;

        inline void setPosition(float2 xy) { spacialInfo.pos = xy; }
        inline void setRandomPosition(float2 width_height_limits) {
            std::default_random_engine generator;
            std::uniform_real_distribution<float> distribution(width_height_limits.x, width_height_limits.y);
            float genX = distribution(generator);
            float genY = distribution(generator);
            spacialInfo.pos = float2(genX, genY);
        }
    };

    namespace forces {
        class Force {
            public:
            using ptr_t = std::unique_ptr<Force>;
            virtual float2 produceSteeringVector(Boid::BoidFlock const& flock, Boid::ptr_t const& actor) const = 0;
            virtual ~Force() = default;
        };

        class SeekForce {
            public:
            virtual float2 produceSteeringVector(Boid::BoidFlock const& flock, Boid::ptr_t const& actor) const {
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
                forces.push_back(std::make_unique<SpecializedForce>(constructorArguments...));
            }
        };
    }

    class MovementSystem : public ecs::System {
        public:
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
    };

    class BoidModel {
        public:
        float2 modelDimensions;

        BoidModel() = delete;
        BoidModel(float2 worldDimensions) : modelDimensions(worldDimensions) {};

        inline void updateModel(float frametime) {}
        inline void renderModel() const {}
    };
}

#endif