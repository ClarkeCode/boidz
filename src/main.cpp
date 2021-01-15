#include "raylib.h"
#include "lib/game_concepts.hpp"
#include "src/boid_model.hpp"

Camera2D setupCamera(linalg::aliases::float2& gw) {
    Camera2D camera {0};
    camera.target = Vector2{ (float)gw.x/2, (float)gw.y/2 };
    camera.offset = Vector2{ (float)gw.x/2, (float)gw.y/2 };
    camera.rotation = 0.0f;
    camera.zoom = 1.0f;
    return camera;
}

void updateGameState(boid::BoidModel& model){
    float frameTime = GetFrameTime();
    model.updateModel(frameTime);
}

void drawGameState(Camera2D& camera, boid::BoidModel& model) {
        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();
            ClearBackground(RAYWHITE);
            
            BeginMode2D(camera);

                model.renderModel();

            EndMode2D();

        EndDrawing();
        //----------------------------------------------------------------------------------
}

int main(int argc, char* argv[]) {
    // Initialization
    //--------------------------------------------------------------------------------------
    using namespace linalg::aliases;
    float2 gameWorld(800, 450);

    InitWindow(gameWorld.x, gameWorld.y, "Boidz");

    Camera2D camera = setupCamera(gameWorld);
    
    boid::BoidModel gameModel(gameWorld);
    
    SetTargetFPS(60); // Set game to run at 60 frames-per-second
    //--------------------------------------------------------------------------------------
    while (!IsWindowReady()) {
        continue;
    }
    // Main game loop
    while (!WindowShouldClose()) { // Detect window close button or ESC key
        updateGameState(gameModel);

        drawGameState(camera, gameModel);
    }

    // De-Initialization
    //--------------------------------------------------------------------------------------
    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}