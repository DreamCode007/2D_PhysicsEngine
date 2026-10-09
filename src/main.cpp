#include "raylib.h"
#include "physics/PhysicsWorld.h"

int main()
{
    InitWindow(1080, 720, "Physics Sandbox");
    SetTargetFPS(60);

    PhysicsWorld world;

  
    RigidBody ball1(Vec2(200.0f, 100.0f), 1.0f, 20.0f, 0.3f);
    RigidBody ball2(Vec2(500.0f, 100.0f), 2.0f, 35.0f, 0.7f);
    //RigidBody ball3(Vec2(800.0f, 50.0f), 5.0f, 50.0f, 0.95f);

    ball1.SetVelocity(Vec2(400.0f, 0.0f));
    ball2.SetVelocity(Vec2(-450.0f, 0.0f));
    //ball3.SetVelocity(Vec2(-100.0f, 100.0f));

    world.AddBody(ball1);
    world.AddBody(ball2);
    //world.AddBody(ball3);

    world.SetGround(650.0f, 0.75f);

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();


        world.Step(dt);

        BeginDrawing();
        ClearBackground(BLACK);
        DrawLine(
            0,
            650,
            1080,
            650,
            GRAY
        );

        const auto& bodies = world.GetBodies();

        for (int i = 0; i < bodies.size(); i++)
        {
            const Vec2& p = bodies[i].GetPosition();

            Color color = (i == 0) ? RED :
                          (i == 1) ? GREEN : BLUE;

            DrawCircleV(
                Vector2{p.x, p.y},
                bodies[i].GetRadius(),
                color
            );
        }

        DrawText("Three independent rigid bodies", 20, 20, 20, WHITE);
        DrawFPS(20, 50);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
