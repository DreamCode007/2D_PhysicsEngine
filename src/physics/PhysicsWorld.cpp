#include "PhysicsWorld.h"
#include "Shape.h"
#include "Circle.h"
#include "Box.h"

PhysicsWorld::PhysicsWorld() : gravity(0.0f, 500.0f), groundY(650.0f),
      restitution(0.75f){}

void PhysicsWorld::AddBody(
    RigidBody body)
{
    bodies.push_back(std::move(body));
}


std::vector<RigidBody>& PhysicsWorld::GetBodies()
{
    return bodies;
}

void PhysicsWorld::SetGround(
    float gY,
    float r
)
{
    groundY = gY;

    if (restitution < 0.0f)
        restitution = 0.0f;

    if (restitution > 1.0f)
        restitution = 1.0f;

    restitution = r;
}


void PhysicsWorld::Step(float dt)
{
    for (RigidBody& body : bodies)
    {
   
        Vec2 gravityForce = gravity * body.GetMass();

        body.ApplyForce(gravityForce);
        body.Integrate(dt);

        body.GroundCollision(
            groundY
        );
    }

    for (int i = 0; i < bodies.size(); ++i)
    {
        for (int j = i + 1; j < bodies.size(); ++j)
        {
            bodies[i].Collision(bodies[j]);
        }
    }
}