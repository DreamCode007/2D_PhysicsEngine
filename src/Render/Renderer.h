#pragma once

#include "raylib.h"
#include "../physics/PhysicsWorld.h"
#include "../physics/GPUPhysicsWorld.h"

class Renderer
{
private:
    Color backgroundColor;
    Color groundColor;

public:
    Renderer();

    void Draw(PhysicsWorld& world);
    void DrawGPU(GPUPhysicsWorld& gpuWorld);
};
