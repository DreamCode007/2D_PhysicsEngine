#pragma once

#include <vector>

#include "RigidBody.h"

class PhysicsWorld
{
private:
    bool gravityEnabled = true;
    std::vector<RigidBody> bodies;
    Vec2 gravity;
    float groundY;
    float restitution;


public:
    PhysicsWorld();

    void AddBody(RigidBody body);

    void Step(float dt);
    void ToggleGravity();
    bool isGravityEnabled() const;

    std::vector<RigidBody>& GetBodies();
    const std::vector<RigidBody>& GetBodies() const;

    void SetGround(float gY, float r);
};