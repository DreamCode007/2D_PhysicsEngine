#include "Renderer.h"

#include "../physics/Circle.h"
#include "../physics/Box.h"

Renderer::Renderer()
    : backgroundColor(BLACK),
      groundColor(GRAY)
{
}

void Renderer::Draw(const PhysicsWorld& world)
{
    BeginDrawing();
    ClearBackground(backgroundColor);

    DrawLine(0, 650, GetScreenWidth(), 650, groundColor);

    const auto& bodies = world.GetBodies();

    for (const RigidBody& body : bodies)
    {
        const Vec2& position = body.GetPosition();
        const Shape& shape = body.GetShape();

        if (shape.GetType() == ShapeType::Circle)
        {
            const auto& circle =
                static_cast<const Circle&>(shape);

            DrawCircleV(
                Vector2{position.x, position.y},
                circle.GetRadius(),
                RED
            );
        }
        else if (shape.GetType() == ShapeType::Box)
        {
            const auto& box =
                static_cast<const Box&>(shape);

            Rectangle rectangle{
                position.x - box.GetWidth() / 2.0f,
                position.y - box.GetHeight() / 2.0f,
                box.GetWidth(),
                box.GetHeight()
            };

            DrawRectangleRec(rectangle, GREEN);
        }
    }

    DrawText("2D Physics Engine", 20, 20, 22, WHITE);
    DrawFPS(20, 50);

    EndDrawing();
}