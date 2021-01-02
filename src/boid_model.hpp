#ifndef GAME_MODEL_BOID
#define GAME_MODEL_BOID
#include "raylib.h"
#include "lib/game_concepts.hpp"
#include <vector>

namespace boid {
    class Boid : public concept::PhysicsObject, public concept::DrawableObject {
        public:
        Boid();

        virtual void drawObject() override;
    };

    class BoidModel : public concept::Model {
        public:
        concept::GameWorld* worldInfo;
        std::vector<Boid> boids;

        BoidModel(concept::GameWorld* gw);

        void renderModel();
    };
}

#endif