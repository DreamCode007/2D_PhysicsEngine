#include "Renderer.h"
#include "string"
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
        std::make_unique<Circle>(2.0f),
        2.0f,
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
    std::string object_Num = std::to_string(bodies.size());
    DrawText(object_Num.c_str(), 1000, 20, 22, WHITE);
    DrawText("2D Physics Engine [CPU Mode]", 20, 20, 22, WHITE);
    DrawText("Press [TAB] to switch to GPU Parallel Mode", 20, 75, 18, GRAY);
    DrawFPS(20, 50);

    EndDrawing();
}

void GrabObjectGPU(GPUPhysicsWorld& gpuWorld)
{
    static int grabbedIndex = -1;
    static Vec2 offset(0.0f, 0.0f);
    static Vec2 previousMouse(0.0f, 0.0f);
    static Vec2 dragVelocity(0.0f, 0.0f);

    Vector2 mouse = GetMousePosition();
    Vec2 mousePos(mouse.x, mouse.y);

    const auto& pos = gpuWorld.GetPositions();
    const auto& angles = gpuWorld.GetAngles();
    const auto& shapeTypes = gpuWorld.GetShapeTypes();
    const auto& halfExtents = gpuWorld.GetHalfExtents();
    int count = gpuWorld.GetBodyCount();

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
    {
        grabbedIndex = -1;
        for (int i = count - 1; i >= 0; --i)
        {
            float bx = pos[i * 2];
            float by = pos[i * 2 + 1];
            float dx = mousePos.x - bx;
            float dy = mousePos.y - by;
            int shape = (i < (int)shapeTypes.size()) ? shapeTypes[i] : 0;
            float hw = (i * 2 < (int)halfExtents.size()) ? halfExtents[i * 2] : 20.0f;
            float hh = (i * 2 + 1 < (int)halfExtents.size()) ? halfExtents[i * 2 + 1] : 20.0f;

            bool inside = false;
            if (shape == 0) // Circle
            {
                inside = (dx * dx + dy * dy) <= (hw * hw);
            }
            else // Box
            {
                float ang = (i < (int)angles.size()) ? angles[i] : 0.0f;
                float c = std::cos(ang);
                float s = std::sin(ang);
                float localX = dx * c + dy * s;
                float localY = -dx * s + dy * c;
                inside = (std::abs(localX) <= hw) && (std::abs(localY) <= hh);
            }

            if (inside)
            {
                grabbedIndex = i;
                offset = Vec2(bx - mousePos.x, by - mousePos.y);
                break;
            }
        }
    }

    if (grabbedIndex >= 0 && grabbedIndex < count)
    {
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
        {
            float dt = GetFrameTime();
            if (dt > 0.0001f)
            {
                dragVelocity = Vec2(
                    (mousePos.x - previousMouse.x) / dt,
                    (mousePos.y - previousMouse.y) / dt
                );
            }

            gpuWorld.SetBodyPosition(grabbedIndex, mousePos.x + offset.x, mousePos.y + offset.y);
            gpuWorld.SetBodyVelocity(grabbedIndex, 0.0f, 0.0f);
            previousMouse = mousePos;
        }

        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
        {
            gpuWorld.SetBodyVelocity(grabbedIndex, dragVelocity.x, dragVelocity.y);
            grabbedIndex = -1;
        }
    }
}

void Renderer::DrawGPU(GPUPhysicsWorld& gpuWorld)
{
    BeginDrawing();
    ClearBackground(backgroundColor);

    DrawLine(0, 650, GetScreenWidth(), 650, groundColor);

    GrabObjectGPU(gpuWorld);

    Vector2 mouse = GetMousePosition();
    if (IsKeyDown(KEY_ONE))
    {
        gpuWorld.SpawnCircle(mouse.x, mouse.y, 2.0f, 40.0f);
    }
    if (IsKeyPressed(KEY_TWO))
    {
        gpuWorld.SpawnBox(mouse.x, mouse.y, 44.0f, 28.0f, 50.0f);
    }
    if (IsKeyPressed(KEY_R))
    {
        gpuWorld.Reset(gpuWorld.GetBodyCount());
    }

    const auto& pos = gpuWorld.GetPositions();
    const auto& angles = gpuWorld.GetAngles();
    const auto& shapeTypes = gpuWorld.GetShapeTypes();
    const auto& halfExtents = gpuWorld.GetHalfExtents();
    int count = gpuWorld.GetBodyCount();

    for (int i = 0; i < count; ++i)
    {
        float x = pos[i * 2];
        float y = pos[i * 2 + 1];
        int shape = (i < (int)shapeTypes.size()) ? shapeTypes[i] : 0;
        float hw = (i * 2 < (int)halfExtents.size()) ? halfExtents[i * 2] : 20.0f;
        float hh = (i * 2 + 1 < (int)halfExtents.size()) ? halfExtents[i * 2 + 1] : 20.0f;
        float ang = (i < (int)angles.size()) ? angles[i] : 0.0f;

        if (shape == 0) // Circle
        {
            DrawCircleV(Vector2{x, y}, hw, RED);
            // Draw rotation line
            float cx = x + std::cos(ang) * hw;
            float cy = y + std::sin(ang) * hw;
            DrawLine(x, y, cx, cy, WHITE);
        }
        else // Box
        {
            Rectangle rect{x, y, hw * 2.0f, hh * 2.0f};
            Vector2 origin{hw, hh};
            float angleDeg = ang * (180.0f / PI);
            DrawRectanglePro(rect, origin, angleDeg, BLUE);
        }
    }

    std::string countStr = "GPU Bodies: " + std::to_string(count);
    DrawText(countStr.c_str(), 860, 20, 22, GREEN);
    DrawText("2D Physics Engine [GPU SAT Parallel - RTX 3050]", 20, 20, 22, WHITE);
    DrawText("[1] Spawn Circle | [2] Spawn Box | Left Click & Drag | [TAB] Switch to CPU", 20, 75, 18, LIGHTGRAY);
    DrawFPS(20, 50);

    EndDrawing();
}