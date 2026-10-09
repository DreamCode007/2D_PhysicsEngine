#pragma once
#include "../math/Vec2.h"

class RigidBody{
    private:
        Vec2 position;
        Vec2 velocity;
        Vec2 force;
        Vec2 acceleration;
        float radius;
        float mass;
        float restitution;
    public:
        RigidBody();
        
        RigidBody(
        const Vec2& position,
        float mass,
        float radius = 25.0f,
        float restitution = 0.75f
        );

        void ApplyForce(const Vec2& force);

        void Integrate(float dt);


        void SetVelocity(const Vec2& velocity);

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

        


        void GroundCollision(
            float groundY
        ) ;

        bool isGrounded(float GroundY);

        void Collision(RigidBody& other);
};