#include "raylib.h"

int main() {
    InitWindow(800, 600, "Planar Arm IK + PID Simulator");
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(RAYWHITE);
        DrawText("Dziala! CMake + Raylib gotowe.", 220, 280, 20, DARKGRAY);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}