#include "boid_model.hpp"
#include "raymath.h"

using namespace extensions;

boid::Boid::Boid(BoidModel* bm) : model_ptr(bm), doRenderDebug(0) {
    float radAngle = DEG2RAD * GetRandomValue(0, 360);
    vel = Vector2Scale(Vector2UnitVectorFromRadian(radAngle), 100);
}

void boid::Boid::drawObject() {
    //Triangle
    Vector2 vertex1, vertex2, vertex3; //CCW order
    Vector2 velDirection = Vector2Scale(Vector2Normalize(vel), 20);
    Vector2 tailPos = Vector2Add(pos, Vector2Scale(Vector2Negate(Vector2Normalize(velDirection)), 10));
    vertex1 = Vector2Add(pos, velDirection);
    vertex2 = Vector2Add(tailPos, Vector2Divide(Vector2PerpendicularCCW(velDirection), 2));
    vertex3 = Vector2Add(tailPos, Vector2Divide(Vector2PerpendicularCW(velDirection), 2));
    DrawTriangle(vertex1, vertex2, vertex3, GRAY);
    DrawTriangleLines(vertex1, vertex2, vertex3, BLACK);


    if (doRenderDebug) {
        Vector2 velTarget = Vector2Add(pos, vel);
        DrawLineEx(pos, velTarget, 1, BLACK);
        DrawCircle(velTarget.x, velTarget.y, 2.5, RED);
        DrawCircle(pos.x, pos.y, 2.5, BLUE);
        DrawCircleLines(pos.x, pos.y, AlignmentBehaviour().neighbourhoodRadius, BLACK);
        DrawLineEx(pos, Vector2Add(pos, AlignmentBehaviour().steeringDirection(this, model_ptr->boids)), 1, GREEN);
        if (Vector2Length(CohesionBehaviour().steeringDirection(this, model_ptr->boids)) != 0)
            DrawLineEx(pos, CohesionBehaviour().steeringDirection(this, model_ptr->boids), 1, PINK);
    }
}

void boid::Boid::updatePosition(float fraction) {
    AlignmentBehaviour align;
    // Vector2 alignVect = align.steeringDirection(this, model_ptr->boids);
    vel = Vector2Add(vel, Vector2MultiplyScalar(acc, fraction));
    // if (Vector2Length(alignVect) != 0)
    //     vel = Vector2Lerp(vel, Vector2ScaleToLength(alignVect, Vector2Length(vel)), fraction);
    pos = Vector2Add(pos, Vector2MultiplyScalar(vel, fraction));
}

float boid::AlignmentBehaviour::neighbourhoodRadius = 100.0f;
Vector2 boid::AlignmentBehaviour::steeringDirection(Boid* boid, std::vector<Boid>& boids) {
    int neighbourBoids = 0;
    Vector2 averageHeading = Vector2Zero();
    for (Boid& b : boids) {
        //Ignore the same boid
        if (&b == boid) continue;
        if (Vector2Length(Vector2Subtract(b.pos, boid->pos)) < neighbourhoodRadius) {
            neighbourBoids += 1;
            averageHeading = Vector2Add(averageHeading, b.vel);
        }
    }
    return Vector2Divide(averageHeading, neighbourBoids);
}

float boid::CohesionBehaviour::neighbourhoodRadius = 100.0f;
Vector2 boid::CohesionBehaviour::steeringDirection(Boid* boid, std::vector<Boid>& boids) {
    int neighbourBoids = 0;
    Vector2 averagePosition = Vector2Zero();
    for (Boid& b : boids) {
        //Ignore the same boid
        if (&b == boid) continue;
        if (Vector2Length(Vector2Subtract(b.pos, boid->pos)) < neighbourhoodRadius) {
            neighbourBoids += 1;
            averagePosition = Vector2Add(averagePosition, b.pos);
        }
    }
    if (neighbourBoids == 0) return Vector2Zero();
    return Vector2Divide(averagePosition, neighbourBoids);
}


boid::BoidModel::BoidModel(concept::GameWorld* gw) {
    worldInfo = gw;
    boids = std::vector<Boid>();
    rules = std::vector<SteeringBehaviour>();
    rules.push_back(AlignmentBehaviour());
    steeringPercent = 0.1f;

    for (int x = 0; x < 10; x++)
        boids.push_back(Boid(this));
    boids[0].pos = Vector2 {gw->width/2, gw->height/2};
    boids[0].doRenderDebug = true;
}

void boid::BoidModel::renderModel() {
    for (Boid& b : boids) {
        b.drawObject();
    }
}

void boid::BoidModel::updateModel(float fraction) {
    //update steering
    // for (Boid& b : boids) {
    //     int rulesApplied = 0;
    //     Vector2 steeringVector = Vector2Zero();
    //     for (SteeringBehaviour& rule : rules) {
    //         rulesApplied += 1;
    //         Vector2Add(steeringVector, Vector2MultiplyScalar(rule.steeringDirection(&b, boids), rule.weight));
    //     }
    //     steeringVector = Vector2Divide(steeringVector, rulesApplied);

    //     if (Vector2Length(steeringVector) != 0)
    //         b.vel = Vector2Lerp(b.vel, steeringVector, fraction * steeringPercent);
    // }
    
    for (Boid& b : boids) {
        b.updatePosition(fraction);
        if (b.pos.y < 0) { b.pos.y = worldInfo->height; }
        if (b.pos.y > worldInfo->height) { b.pos.y = 0; }
        if (b.pos.x < 0) { b.pos.x = worldInfo->width; }
        if (b.pos.x > worldInfo->width) { b.pos.x = 0; }
    }
}