#include "raylib.h"
#include "lib/raylib_extensions.hpp"
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

void updateGameState(boid::BoidModel& model) {
    float frameTime = GetFrameTime();
    model.updateModel(frameTime);
    
    // //P1 controls
    // if (IsKeyDown(KEY_W))
    //     model.P1Paddle.updatePaddle(frameTime * model.P1Paddle.speed * -1, model.BottomWall.getCollisionBox().y, model.TopWall.getCollisionBox().height);
    // if (IsKeyDown(KEY_S))
    //     model.P1Paddle.updatePaddle(frameTime * model.P1Paddle.speed, model.BottomWall.getCollisionBox().y, model.TopWall.getCollisionBox().height);

    // //P2 controls
    // if (IsKeyDown(KEY_UP))
    //     model.P2Paddle.updatePaddle(frameTime * model.P2Paddle.speed * -1, model.BottomWall.getCollisionBox().y, model.TopWall.getCollisionBox().height);
    // if (IsKeyDown(KEY_DOWN))
    //     model.P2Paddle.updatePaddle(frameTime * model.P2Paddle.speed, model.BottomWall.getCollisionBox().y, model.TopWall.getCollisionBox().height);
    
    // //Start ball movement
    // if (IsKeyPressed(KEY_R)) {
    //     if (model.canBallMove) {
    //         model.PongBall.xyPosition = Vector2{model.worldInfo->width/2.0f, model.worldInfo->height/2.0f};
    //         model.PongBall.setDirection();
    //     }
    //     model.canBallMove = true;
    // }

    // model.PongBall.updateBall(frameTime, model);
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
    concept::GameWorld gameWorld(800, 450);

    InitWindow(gameWorld.width, gameWorld.height, "Boidz");

    Camera2D camera = setupCamera(gameWorld);
    
    boid::BoidModel gameModel(&gameWorld);
    
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