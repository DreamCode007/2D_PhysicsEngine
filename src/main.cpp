#include "raylib.h"
#include "physics/PhysicsWorld.h"
#include "physics/Circle.h"
#include "physics/Box.h"
#include "Render/Renderer.h"

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

    // Rectangle body
    RigidBody rectangle(
        Vec2(450.0f, 100.0f),
        60.0f,
        std::make_unique<Box>(100.0f, 50.0f),
        25.0f,
        0.5f
    );

    rectangle.SetVelocity(Vec2(-80.0f, 0.0f));

    // Transfer ownership to the physics world.
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
