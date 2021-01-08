#ifndef GAME_MODEL_BOID
#define GAME_MODEL_BOID
#include <vector>
#include "lib/ecs.hpp"
#include "lib/linalg.h"

namespace boid {
    using namespace linalg::aliases;
    
    class SpacialComponent : public ecs::Component {
        public:
        float2 pos, vel, acc;
        
        SpacialComponent() : pos(0.0f), vel(0.0f), acc(0.0f) {}
        SpacialComponent(float2 position, float2 velocity, float2 acceleration) : pos(position), vel(velocity), acc(acceleration) {}
    };

    class VisualComponent : public ecs::Component {
        public:
        int debugLevel;
        bool isDisplayable, showDebugInfo;

        VisualComponent() : isDisplayable(true), showDebugInfo(false), debugLevel(0) {}
    };

    class Boid : public ecs::Entity {
        public:
        Boid() : ecs::Entity(2) {
            components.push_back((ecs::Component::ptr_t)new SpacialComponent);
            components.push_back((ecs::Component::ptr_t)new VisualComponent);
        }
    };

    class MovementSystem : public ecs::System {
        public:
        //TODO: May need to keep screensize information as member attributes that are initialized at construction
        virtual void process(ecs::Entity::EntityContainer const& entities) override {
            //TODO: Implement movement mehaviour of boids with SpacialComponents, rules for alignment, separation, cohesion may need to be a separate system
        }
    };

    class VirtualSystem : public ecs::System {
        public:
        virtual void process(ecs::Entity::EntityContainer const& entities) override {
            //TODO: Implement drawing behaviours of boids with SpacialComponents and VisualComponents
        }
    };

}

#endif