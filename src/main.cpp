#include "raylib.h"

int main(void)
{
    const int screenWidth = 1080;
    const int screenHeight = 720;
    const float g = -9.81;
    int x = 300, y = 200;
    float dt = 0;
    InitWindow(screenWidth, screenHeight, "Physics Engine");

    SetTargetFPS(60);

    while (!WindowShouldClose())
    {
        BeginDrawing();

        ClearBackground(BLACK);
        DrawCircle(x, y, 30, WHITE);
        y = y - (g*dt)/2;
        dt += 0.04;
        if(IsMouseButtonDown(0)){
            x = GetMouseX();
            y = GetMouseY();
            dt = 0;
        }
       

        DrawText("First Window", 300, 200, 30, WHITE);

        EndDrawing();
    }

    CloseWindow();

    return 0;
}