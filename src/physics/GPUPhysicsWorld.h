#pragma once
#include <vector>
#include <memory>
#include "../math/Vec2.h"

class GPUPhysicsWorld {
public:
    GPUPhysicsWorld();
    ~GPUPhysicsWorld();

    void Init(int count, float worldW = 1080.0f, float groundY = 650.0f, float gravityY = 500.0f);
    void Step(float dt);
    
    int GetBodyCount() const;
    const std::vector<float>& GetPositions() const;
    const std::vector<float>& GetAngles() const;
    const std::vector<int>& GetShapeTypes() const;      // 0 = Circle, 1 = Box
    const std::vector<float>& GetHalfExtents() const;   // [2 * count] -> [hw, hh]
    
    void SpawnCircle(float x, float y, float radius = 20.0f, float mass = 50.0f);
    void SpawnBox(float x, float y, float width = 50.0f, float height = 35.0f, float mass = 60.0f);
    
    void SetBodyPosition(int index, float x, float y);
    void SetBodyVelocity(int index, float vx, float vy);
    
    void Reset(int count);

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
