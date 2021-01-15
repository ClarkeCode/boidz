#ifndef GAME_MODEL_BOID
#define GAME_MODEL_BOID
#include <vector>
#include "lib/ecs.hpp"
#include "lib/linalg.h"
#include <random> //randomization
#include <utility> //move
#include "raylib.h" //renderingsystem
#include "iostream" //debug messages
#include <chrono> //random seeding
#include <algorithm> //for_each

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
        inline void setRandomPosition(std::mt19937& generator, 
            std::uniform_real_distribution<float>& distributionX, 
            std::uniform_real_distribution<float>& distributionY) {
            float genX = distributionX(generator);
            float genY = distributionY(generator);
            spacialInfo.pos = float2(genX, genY);
        }
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
        //Conversion function to turn linalg vector into a raylib vector
        Vector2 compat(linalg::aliases::float2 const& vect) { return Vector2{vect.x, vect.y}; }
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
            }
        }
    };

    class BoidModel {
        public:
        float2 modelDimensions;

        Boid::Flock flock;

        MovementSystem movementSystem;
        RenderingSystem renderingSystem;

        //Mersenne twister generator
        std::mt19937 generator;
        std::uniform_real_distribution<float> distributionX;
        std::uniform_real_distribution<float> distributionY;

        BoidModel() = delete;
        BoidModel(float2 worldDimensions) : modelDimensions(worldDimensions) {
            //std::random_device rd;
            generator = std::mt19937(std::random_device()());
            
            distributionX = std::uniform_real_distribution<float>(0, modelDimensions.x);
            distributionY = std::uniform_real_distribution<float>(0, modelDimensions.y);

            for (int x = 0; x < 10; x++) {
                flock.push_back(std::move(std::make_shared<Boid>()));
            }
            std::for_each(flock.begin(), flock.end(), [this](Boid::ptr_t& boidp)->void{boidp->setRandomPosition(generator, distributionX, distributionY);});

            flock[0]->spacialInfo.pos = float2(modelDimensions.x/2.0f, modelDimensions.y/2.0f);
            flock[0]->visualInfo.showDebugInfo = true;
            for(auto bp : flock)
                std::cout << bp->spacialInfo.pos.x << "," << bp->spacialInfo.pos.y << std::endl;
        };

        inline void updateModel(float frametime) {}
        inline void renderModel() const { renderingSystem.render(flock); }
    };
}

#endif