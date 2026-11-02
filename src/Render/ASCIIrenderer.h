#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include "../physics/PhysicsWorld.h"
#include "../physics/Circle.h"
#include "../physics/Box.h"

class AsciiRenderer
{
public:
    static const int WIDTH  = 80;  
    static const int HEIGHT = 30;   

    const float WORLD_W = 1080.0f;
    const float WORLD_H = 720.0f;

    void Draw(const PhysicsWorld& world)
    {
     
        std::vector<std::string> grid(HEIGHT, std::string(WIDTH, ' '));


        int groundRow = static_cast<int>((650.0f / WORLD_H) * HEIGHT);
        if (groundRow >= 0 && groundRow < HEIGHT)
        {
            grid[groundRow] = std::string(WIDTH, '=');
        }

        
        for (const auto& body : world.GetBodies())
        {
            const Vec2& pos    = body.GetPosition();
            const Shape& shape = body.GetShape();

            if (shape.GetType() == ShapeType::Circle)
            {
                const auto& circle = static_cast<const Circle&>(shape);
                float r = circle.GetRadius();

                for (int y = 0; y < HEIGHT; ++y)
                {
                    float worldY = (y + 0.5f) / HEIGHT * WORLD_H;
                    for (int x = 0; x < WIDTH; ++x)
                    {
                        float worldX = (x + 0.5f) / WIDTH * WORLD_W;

                        
                        float dx = worldX - pos.x;
                        float dy = (worldY - pos.y) * 1.6f;

                        if (dx * dx + dy * dy <= r * r)
                            grid[y][x] = 'O';
                    }
                }
            }
            else if (shape.GetType() == ShapeType::Box)
            {
                const auto& box = static_cast<const Box&>(shape);
                float hw = box.GetWidth() * 0.5f;
                float hh = box.GetHeight() * 0.5f;

                float cosA = std::cos(-body.GetAngle());
                float sinA = std::sin(-body.GetAngle());

                for (int y = 0; y < HEIGHT; ++y)
                {
                    float worldY = (y + 0.5f) / HEIGHT * WORLD_H;
                    for (int x = 0; x < WIDTH; ++x)
                    {
                        float worldX = (x + 0.5f) / WIDTH * WORLD_W;

                        float dx = worldX - pos.x;
                        float dy = worldY - pos.y;

                        // Rotate cell point into box's local space
                        float localX =  dx * cosA - dy * sinA;
                        float localY =  dx * sinA + dy * cosA;

                        if (std::abs(localX) <= hw && std::abs(localY) <= hh)
                            grid[y][x] = '#';
                    }
                }
            }
        }

        
        std::cout << "\033[H";
        for (int y = 0; y < HEIGHT; ++y)
        {
            std::cout << grid[y] << "\n";
        }
    }
};
