#pragma once
#include "../math/Vec2.h"
#include "Shape.h"
#include <memory>

class RigidBody{
    private:
        Vec2 position;
        Vec2 velocity;
        Vec2 force;
        Vec2 acceleration;
        float radius;
        float mass;
        float restitution;
        std::unique_ptr<Shape> shape;
    public:
        RigidBody();
        
        RigidBody(
        const Vec2& position,
        float mass,
        std::unique_ptr<Shape> shape,
        float radius = 25.0f,
        float restitution = 0.75f
        );

        RigidBody(const RigidBody&) = delete;
        RigidBody& operator=(const RigidBody&) = delete;

        // Bodies can be moved into the physics world.
        RigidBody(RigidBody&&) noexcept = default;
        RigidBody& operator=(RigidBody&&) = default;

        void ApplyForce(const Vec2& force);

        void Integrate(float dt);


        void SetVelocity(const Vec2& velocity);
        void SetPosition(const Vec2& p);

        void SetAcceleration(
            const Vec2& acceleration
        );

        void SetRadius(float radius);
        void SetRestitution(float restitution);



        const Vec2& GetPosition() const;

        const Vec2& GetVelocity() const;

        float GetMass() const;

        const float GetRadius() const;

        const float GetRestitution() const;

        Shape& GetShape();
        const Shape& GetShape() const;


        void GroundCollision(
            float groundY
        );

        bool isGrounded(float GroundY);

        void Collision(RigidBody& other);
};