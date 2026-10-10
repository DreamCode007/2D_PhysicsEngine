#pragma once

#include <vector>

#include "RigidBody.h"

class PhysicsWorld
{
private:

    std::vector<RigidBody> bodies;
    Vec2 gravity;
    float groundY;
    float restitution;


public:
    PhysicsWorld();

    void AddBody(RigidBody body);

    void Step(float dt);

    std::vector<RigidBody>& GetBodies();\

    void SetGround(float gY, float r);
};