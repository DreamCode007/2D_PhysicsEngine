#include "RigidBody.h"
#include <cmath>

RigidBody::RigidBody()
    : position(0.0f, 0.0f),
      velocity(0.0f, 0.0f),
      acceleration(0.0f, 0.0f),
      mass(1.0f),
      radius(25.0f),
      restitution(0.75f)
{
}


RigidBody::RigidBody(
    const Vec2& position,
    float mass,
    std::unique_ptr<Shape> shape,
    float radius,  
    float restitution
    )
    : position(position),
      velocity(0.0f, 0.0f),
      mass(mass),
      force(0.0f, 0.0f),
      acceleration(0.0f, 0.0f),
      shape(std::move(shape)),
      radius(radius),
      restitution(
        restitution < 0.0f ? 0.0f : 
        (restitution > 1.0f ? 1.0f : restitution)
      )
      
{
}

void RigidBody::ApplyForce(const Vec2& f){
    force += f;
}

void RigidBody::Integrate(float dt)
{
   if(dt <= 0.0f)
    return;

    Vec2 acceleration = force/mass;


    velocity += acceleration * dt;


    position += velocity * dt;

    force = Vec2{0.0f, 0.0f};
}


void RigidBody::SetVelocity(
    const Vec2& vel) 
{
    velocity = vel;
}


void RigidBody::SetAcceleration(
    const Vec2& accel)
{
    acceleration = accel;
}

void RigidBody::SetRadius(float r){
    if(r > 0.0f)
        radius = r;
}

void RigidBody::SetRestitution(float r){
    if(r < 0.0f){
        restitution = 0.0f;
    }
        

    if(r > 1.0f){
        restitution = 1.0f;
    }

    restitution = r;
        
}

void RigidBody::SetPosition(const Vec2& v) 
{
    position = v;
}

const Vec2& RigidBody::GetPosition() const
{
    return position;
}


const Vec2& RigidBody::GetVelocity() const
{
    return velocity;
}


float RigidBody::GetMass() const
{
    return mass;
}

const float RigidBody::GetRadius() const{
    return radius;
}

const float RigidBody::GetRestitution() const{
    return restitution;
}

void RigidBody::GroundCollision(
    float groundY
)
{
    radius = GetRadius();
    if(radius <= 0.0f) return;

    if (position.y + radius >= groundY)
    {
        
        position.y = groundY - radius;

        
        if (velocity.y > 0.0f)
        {
            velocity.y *= -restitution;
        }
    }
}

bool RigidBody::isGrounded(
    float groundY
)
{
    return position.y + radius >= groundY - 0.01f;
}

Shape& RigidBody::GetShape()
{
    return *shape;
}

const Shape& RigidBody::GetShape() const
{
    return *shape;
}
void RigidBody::Collision(RigidBody& other){

    Vec2 normal = other.position - position;

    float distance = normal.x * normal.x + normal.y * normal.y;

    float totalRadius = GetRadius() + other.GetRadius();

    if(distance >= totalRadius*totalRadius) return;

    // avoid division by zero
    distance = std::sqrt(distance);

    if(distance > 0.0001f){
        normal = normal/distance;
    }
    else{
        normal = Vec2(1.0f, 1.0f);
        distance = 0.0f;
    }

    const float invMassA = 1.0f / mass;
    const float invMassB = 1.0f / other.mass;
    const float invMassSum = invMassA + invMassB;


    const float penetration = totalRadius - distance;
    const float slop = 0.01f;
    const float percent = 0.8f;

    const float correctionMagnitude =
        percent * std::max(penetration - slop, 0.0f)
        / invMassSum;

    const Vec2 correction = normal * correctionMagnitude;

    position -= correction * invMassA;
    other.position += correction * invMassB;


    const Vec2 relativeVelocity =
        other.velocity - velocity;

    const float velocityAlongNormal =
        relativeVelocity.x * normal.x +
        relativeVelocity.y * normal.y;

    // If the bodies are separating, don't apply another impulse.
    if (velocityAlongNormal >= 0.0f)
        return;

    ///Calculate the collision impulse.
    const float r =
        std::min(restitution, other.restitution);

    const float impulseMagnitude =
        -(1.0f + r) * velocityAlongNormal
        / invMassSum;

    const Vec2 impulse = normal * impulseMagnitude;


    velocity -= impulse * invMassA;
    other.velocity += impulse * invMassB;

    
    
}