#include "boid_model.hpp"
#include "raymath.h"

using namespace extensions;

boid::Boid::Boid() {
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


    Vector2 velTarget = Vector2Add(pos, vel);
    DrawLineEx(pos, velTarget, 1, BLACK);
    DrawCircle(velTarget.x, velTarget.y, 2.5, RED);
    DrawCircle(pos.x, pos.y, 2.5, BLUE);
}

void boid::Boid::updatePosition(float fraction) {
    vel = Vector2Add(vel, Vector2MultiplyScalar(acc, fraction));
    pos = Vector2Add(pos, Vector2MultiplyScalar(vel, fraction));
}


boid::BoidModel::BoidModel(concept::GameWorld* gw) {
    worldInfo = gw;
    boids = std::vector<Boid>();

    for (int x = 0; x < 10; x++)
        boids.push_back(Boid());
    boids[0].pos = Vector2 {gw->width/2, gw->height/2};
}

void boid::BoidModel::renderModel() {
    for (Boid& b : boids) {
        b.drawObject();
    }
}

void boid::BoidModel::updateModel(float fraction) {
    for (Boid& b : boids) {
        b.updatePosition(fraction);
        if (b.pos.y < 0) { b.pos.y = worldInfo->height; }
        if (b.pos.y > worldInfo->height) { b.pos.y = 0; }
        if (b.pos.x < 0) { b.pos.x = worldInfo->width; }
        if (b.pos.x > worldInfo->width) { b.pos.x = 0; }
    }
}