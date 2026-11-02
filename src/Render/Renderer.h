#pragma once

#include "raylib.h"
#include "../physics/PhysicsWorld.h"

class Renderer
{
private:
    Color backgroundColor;
    Color groundColor;

public:
    Renderer();

    void Draw(PhysicsWorld& world);
};