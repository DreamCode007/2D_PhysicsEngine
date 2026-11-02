#include "Renderer.h"

#include "../physics/Circle.h"
#include "../physics/Box.h"

Renderer::Renderer()
    : backgroundColor(BLACK),
      groundColor(GRAY)
{
}

void SpawnCircle(PhysicsWorld& world, Vec2 p){
    RigidBody ball(
        p,
        50.0f,
        std::make_unique<Circle>(0.9f),
        0.9f,
        0.8f
    );

    world.AddBody(std::move(ball));
}

void SpawnBox(PhysicsWorld& world, Vec2 p){
    RigidBody rect(
        p,
        60.0f,
        std::make_unique<Box>(100.0f, 50.0f),
        25.0f,
        0.5f
    );

    world.AddBody(std::move(rect));
}

void GrabObject(PhysicsWorld& world)
{
    static RigidBody* grabbedBody = nullptr;
    static Vec2 offset(0.0f, 0.0f);
    static Vec2 previousMouse(0.0f, 0.0f);
    static Vec2 dragVelocity(0.0f, 0.0f);

    Vector2 mouse = GetMousePosition();
    Vec2 mousePos(mouse.x, mouse.y);

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
    {
        grabbedBody = nullptr;

        auto& bodies = world.GetBodies();

        
        for (auto it = bodies.rbegin(); it != bodies.rend(); ++it)
        {
            RigidBody& body = *it;
            const Vec2& pos = body.GetPosition();

            float dx = mousePos.x - pos.x;
            float dy = mousePos.y - pos.y;

            const Shape& shape = body.GetShape();
            bool inside = false;

            if (shape.GetType() == ShapeType::Circle)
            {
                const auto& circle =
                    static_cast<const Circle&>(shape);

                float r = circle.GetRadius();
                inside = dx * dx + dy * dy <= r * r;
            }
            else if (shape.GetType() == ShapeType::Box)
            {
                const auto& box =
                    static_cast<const Box&>(shape);

                float angle = body.GetAngle();
                float c = std::cos(angle);
                float s = std::sin(angle);

                
                float localX = dx * c + dy * s;
                float localY = -dx * s + dy * c;

                inside =
                    std::abs(localX) <= box.GetWidth() * 0.5f &&
                    std::abs(localY) <= box.GetHeight() * 0.5f;
            }

            if (inside)
            {
                grabbedBody = &body;

                offset = Vec2(
                    pos.x - mousePos.x,
                    pos.y - mousePos.y
                );

                break;
            }
        }
    }

    if (grabbedBody != nullptr)
    {
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
        {
            const float dt = GetFrameTime();

            if (dt > 0.0001f)
            {
                dragVelocity = Vec2(
                    (mousePos.x - previousMouse.x) / dt,
                    (mousePos.y - previousMouse.y) / dt
                );
            }

            grabbedBody->SetPosition(Vec2(
                mousePos.x + offset.x,
                mousePos.y + offset.y
            ));

            grabbedBody->SetVelocity(Vec2(0.0f, 0.0f));

            previousMouse = mousePos;
        }

        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
        {
            grabbedBody->SetVelocity(dragVelocity);
            grabbedBody = nullptr;
        }
    }
}

void Renderer::Draw(PhysicsWorld& world)
{
    BeginDrawing();
    ClearBackground(backgroundColor);

    DrawLine(0, 650, GetScreenWidth(), 650, groundColor);

    auto& bodies = world.GetBodies();
    GrabObject(world);
    
    if (IsKeyPressed(KEY_G))
    {
        DrawText(
        "Gravity Toggled",
        (GetScreenWidth() - 15) / 2,
        20,
        24,
        GREEN
    );
        world.ToggleGravity();
    }

    for (RigidBody& body : bodies)
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
            const float c = std::cos(body.GetAngle());
            const float s = std::sin(body.GetAngle());
        }
        else if (shape.GetType() == ShapeType::Box)
        {
            const auto& box =
                static_cast<const Box&>(shape);

            Rectangle rectangle{
                position.x,
                position.y,
                box.GetWidth(),
                box.GetHeight()
            };
            Vector2 origin = {
                box.GetWidth() * 0.5f,
                box.GetHeight() * 0.5f
            };

            const float angleDegrees = body.GetAngle() * (180.0f / PI);

            DrawRectanglePro(
                rectangle,
                origin,
                angleDegrees,
                BLUE
            );
        }
    }
    const Vector2 mouse = GetMousePosition();
    const Vec2 spawnPosition(mouse.x, mouse.y);

    if (IsKeyDown(KEY_ONE))
    {
        SpawnCircle(world, spawnPosition);
    }

    if (IsKeyPressed(KEY_TWO))
    {
        SpawnBox(world, spawnPosition);
    }

    DrawText("2D Physics Engine", 20, 20, 22, WHITE);
    DrawFPS(20, 50);

    EndDrawing();
}