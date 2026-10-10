#include "raylib.h"
#include "physics/PhysicsWorld.h"
#include "physics/Circle.h"
#include "physics/Box.h"

int main()
{
    InitWindow(1080, 720, "Physics Sandbox");
    SetTargetFPS(60);

    PhysicsWorld world;

  
    RigidBody ball(
        Vec2(200.0f, 100.0f),
        50.0f,
        std::make_unique<Circle>(25.0f),
        25.0f,
        0.8f
    );

    ball.SetVelocity(Vec2(100.0f, 0.0f));

    // Rectangle body
    RigidBody rectangle(
        Vec2(500.0f, 100.0f),
        60.0f,
        std::make_unique<Box>(100.0f, 50.0f),
        25.0f,
        0.5f
    );

    rectangle.SetVelocity(Vec2(-30.0f, 0.0f));

    // Transfer ownership to the physics world.
    world.AddBody(std::move(ball));
    world.AddBody(std::move(rectangle));

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
            const Shape& shape = bodies[i].GetShape();
            if (shape.GetType() == ShapeType::Circle)
            {
                const auto& circle =
                    static_cast<const Circle&>(shape);

                DrawCircleV(
                    Vector2{p.x, p.y},
                    circle.GetRadius(),
                    RED
                );
            }
            else if (shape.GetType() == ShapeType::Box)
            {
                const auto& box =
                    static_cast<const Box&>(shape);

                DrawRectangle(
                    static_cast<int>(
                        p.x - box.GetWidth() / 2.0f
                    ),
                    static_cast<int>(
                        p.y - box.GetHeight() / 2.0f
                    ),
                    static_cast<int>(box.GetWidth()),
                    static_cast<int>(box.GetHeight()),
                    GREEN
                );
            }
        }

        

        DrawText("Three independent rigid bodies", 20, 20, 20, WHITE);
        DrawFPS(20, 50);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
