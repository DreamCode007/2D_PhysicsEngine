#include "RigidBody.h"
#include <cmath>
#include "Circle.h"
#include "Box.h"
#include <algorithm>

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
    float bottom = position.y;
    radius = GetRadius();
    if(radius <= 0.0f) return;

    if(shape->GetType() == ShapeType::Circle){

        const auto& circle = static_cast<const Circle&>(*shape);

        radius = circle.GetRadius();
        bottom = position.y + radius;
    }
    else if(shape->GetType() == ShapeType::Box){

        const auto& box = static_cast<const Box&>(*shape);
        bottom = position.y + box.GetHeight()/2.0f;
        
    }
    else{
        return;
    }

    if(bottom < groundY){
        return;
    }

    
    if (bottom >= groundY)
    {
        
        position.y -= bottom - groundY;

        
        if (velocity.y > 0.0f)
        {
            velocity.y *= -restitution;
            if(velocity.y > - 1.0f && velocity.y < 0.0f){
                velocity.y = 0.0f;
            }
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

void RigidBody::ResolveCircleBox(RigidBody& boxBody){
    if (shape->GetType() != ShapeType::Circle ||
        boxBody.shape->GetType() != ShapeType::Box)
    {
        return;
    }
    const auto& circle = static_cast<const Circle&>(*shape);
    const  auto& box = static_cast<const Box&>(*boxBody.shape);

    const float rad = circle.GetRadius();

    const float halfWidth = box.GetWidth()/2;
    const float halfHeight = box.GetHeight()/2;

    const float closestX = std::clamp(
        position.x,
        boxBody.position.x - halfWidth,
        boxBody.position.x + halfWidth
    );

    const float closestY = std::clamp(
        position.y,
        boxBody.position.y - halfHeight,
        boxBody.position.y + halfHeight
    );

    Vec2 normal(
        closestX - position.x,
        closestY - position.y

    );

    float distance = normal.x*normal.x + normal.y*normal.y;

    if(distance >= rad*rad)
        return;

    distance = std::sqrt(distance);

    if (distance < 0.0001f)
    {
        const float left =
            position.x - (boxBody.position.x - halfWidth);
        const float right =
            (boxBody.position.x + halfWidth) - position.x;
        const float top =
            position.y - (boxBody.position.y - halfHeight);
        const float bottom =
            (boxBody.position.y + halfHeight) - position.y;

        const float nearest = std::min(
            std::min(left, right),
            std::min(top, bottom)
        );

        if (nearest == left)
            normal = Vec2(-1.0f, 0.0f);
        else if (nearest == right)
            normal = Vec2(1.0f, 0.0f);
        else if (nearest == top)
            normal = Vec2(0.0f, -1.0f);
        else
            normal = Vec2(0.0f, 1.0f);

        distance = 0.0f;
    }
    else
    {
        normal = normal / distance;
    }

    
    const float invMassA = 1.0f / mass;
    const float invMassB = 1.0f / boxBody.mass;
    const float invMassSum = invMassA + invMassB;

   
    const float penetration = rad - distance;
    const float correctionMagnitude =
        std::max(penetration - 0.01f, 0.0f) * 0.8f/invMassSum;

    const Vec2 correction = normal*correctionMagnitude;

    position -= correction * invMassA;
    boxBody.position += correction * invMassB;

   
    const Vec2 relativeVelocity = boxBody.velocity - velocity;

    const float velocityAlongNormal =
        relativeVelocity.x * normal.x +
        relativeVelocity.y * normal.y;


    if (velocityAlongNormal >= 0.0f)
        return;

    const float resti = std::min(restitution, boxBody.restitution);

    const float impulseMagnitude =
        -(1.0f + resti) * velocityAlongNormal/invMassSum;

    const Vec2 impulse = normal * impulseMagnitude;

    velocity -= impulse*invMassA;
    boxBody.velocity += impulse*invMassB;
    
}