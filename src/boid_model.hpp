#ifndef GAME_MODEL_BOID
#define GAME_MODEL_BOID
#include "raylib.h"
#include "lib/game_concepts.hpp"
#include <vector>

namespace boid {
    class BoidModel;

    class Boid : public concept::PhysicsObject, public concept::DrawableObject {
        public:
        BoidModel* model_ptr;
        bool doRenderDebug;
        Boid(BoidModel*);

        virtual void drawObject() override;
        void updatePosition(float fraction);
    };

    class SteeringBehaviour {
        public:
        float weight;
        SteeringBehaviour() : weight(1) {}
        virtual ~SteeringBehaviour() {}
        virtual Vector2 steeringDirection(Boid* boid, std::vector<Boid>& boids) { return Vector2Zero(); };
    };

    class AlignmentBehaviour : public SteeringBehaviour {
        public:
        static float neighbourhoodRadius;
        AlignmentBehaviour() : SteeringBehaviour() {};
        virtual Vector2 steeringDirection(Boid* boid, std::vector<Boid>& boids);
    };

    class CohesionBehaviour : public SteeringBehaviour {
        public:
        static float neighbourhoodRadius;
        virtual Vector2 steeringDirection(Boid* boid, std::vector<Boid>& boids);
    };

    class BoidModel : public concept::Model {
        public:
        concept::GameWorld* worldInfo;
        float steeringPercent;
        std::vector<Boid> boids;
        std::vector<SteeringBehaviour> rules;

        BoidModel(concept::GameWorld* gw);

        void renderModel();
        void updateModel(float fraction);
    };
}

#endif