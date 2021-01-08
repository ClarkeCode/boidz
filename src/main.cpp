#include "raylib.h"
#include "lib/game_concepts.hpp"
#include "src/boid_model.hpp"

Camera2D setupCamera(concept::GameWorld& gw) {
    Camera2D camera {0};
    camera.target = Vector2{ (float)gw.width/2, (float)gw.height/2 };
    camera.offset = Vector2{ (float)gw.width/2, (float)gw.height/2 };
    camera.rotation = 0.0f;
    camera.zoom = 1.0f;
    return camera;
}

void updateGameState(){
    float frameTime = GetFrameTime();
}

void drawGameState(Camera2D& camera){//, boid::BoidModel& model) {
        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();
            ClearBackground(RAYWHITE);
            
            BeginMode2D(camera);

                //model.renderModel();

            EndMode2D();

        EndDrawing();
        //----------------------------------------------------------------------------------
}

int main(int argc, char* argv[]) {
    // Initialization
    //--------------------------------------------------------------------------------------
    concept::GameWorld gameWorld(800, 450);

    InitWindow(gameWorld.width, gameWorld.height, "Boidz");

    Camera2D camera = setupCamera(gameWorld);
    
    //boid::BoidModel gameModel(&gameWorld);
    
    SetTargetFPS(60); // Set game to run at 60 frames-per-second
    //--------------------------------------------------------------------------------------
    while (!IsWindowReady()) {
        continue;
    }
    // Main game loop
    while (!WindowShouldClose()) { // Detect window close button or ESC key
        //updateGameState(gameModel);

        //drawGameState(camera, gameModel);
    }

    // De-Initialization
    //--------------------------------------------------------------------------------------
    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}