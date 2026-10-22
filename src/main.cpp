#include "raylib.h"
#include "physics/PhysicsWorld.h"
#include "physics/Circle.h"
#include "physics/Box.h"
#include "Render/Renderer.h"

/*
$env:PATH = "C:\msys64\ucrt64\bin;C:\msys64\usr\bin;" + $env:PATH
cmake --build build
*/


int main()
{
    InitWindow(1080, 720, "Physics Sandbox");
    SetTargetFPS(60);

    PhysicsWorld world;

  
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

 
    world.AddBody(std::move(ball));
    world.AddBody(std::move(rectangle));

    world.SetGround(650.0f, 0.75f);

    Renderer renderer;

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();


        world.Step(dt);

        renderer.Draw(world);
    }

    CloseWindow();
    return 0;
}
