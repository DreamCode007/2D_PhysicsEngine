#include "raylib.h"
#include "physics/PhysicsWorld.h"
#include "physics/GPUPhysicsWorld.h"
#include "physics/Circle.h"
#include "physics/Box.h"
#include "Render/Renderer.h"
#include <iostream>

/*
$env:PATH = "C:\msys64\ucrt64\bin;C:\msys64\usr\bin;" + $env:PATH
cmake --build build
*/

int main()
{
    InitWindow(1080, 720, "2D GPU Parallel Physics Engine");
    SetTargetFPS(120);

    // CPU World setup
    PhysicsWorld cpuWorld;
    RigidBody ball(
        Vec2(250.0f, 100.0f),
        50.0f,
        std::make_unique<Circle>(25.0f),
        25.0f,
        0.8f
    );
    ball.SetVelocity(Vec2(250.0f, 0.0f));

    RigidBody rectangle(
        Vec2(450.0f, 100.0f),
        60.0f,
        std::make_unique<Box>(100.0f, 50.0f),
        25.0f,
        0.5f
    );
    rectangle.SetVelocity(Vec2(-80.0f, 0.0f));

    cpuWorld.AddBody(std::move(ball));
    cpuWorld.AddBody(std::move(rectangle));
    cpuWorld.SetGround(650.0f, 0.75f);

    // GPU Parallel World setup (500 bodies on RTX 3050)
    std::cout << "[Main] Initializing GPU Parallel Physics Engine..." << std::endl;
    GPUPhysicsWorld gpuWorld;
    gpuWorld.Init(0, 1080.0f, 650.0f, 500.0f);

    Renderer renderer;
    bool useGPU = true;

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();

        // Switch modes with TAB
        if (IsKeyPressed(KEY_TAB))
        {
            useGPU = !useGPU;
        }

        if (useGPU)
        {
            gpuWorld.Step(dt);
            renderer.DrawGPU(gpuWorld);
        }
        else
        {
            cpuWorld.Step(dt);
            renderer.Draw(cpuWorld);
        }
    }

    CloseWindow();
    return 0;
}
