#include "RigidBody.h"
#include <cmath>
#include "Circle.h"
#include "Box.h"
#include <algorithm>
#include <vector>
#include <limits>
#include <iostream>

RigidBody::RigidBody()
    : position(0.0f, 0.0f),
      velocity(0.0f, 0.0f),
      acceleration(0.0f, 0.0f),
      mass(1.0f),
      radius(25.0f),
      restitution(0.75f)
{
    if (mass <= 0.0f)
    {
        this->mass = 1.0f;
    }

    if (this->shape)
    {
        if (this->shape->GetType() == ShapeType::Circle)
        {
            const auto& circle =
                static_cast<const Circle&>(*this->shape);

            const float r = circle.GetRadius();

            inertia = 0.5f * this->mass * r * r;
        }
        else if (this->shape->GetType() == ShapeType::Box)
        {
            const auto& box = static_cast<const Box&>(*this->shape);

            const float w = box.GetWidth();
            const float h = box.GetHeight();

            inertia = this->mass * (w * w + h * h) / 12.0f;
        }
    }
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
    if (mass <= 0.0f)
    {
        this->mass = 1.0f;
    }

    if (this->shape)
    {
        if (this->shape->GetType() == ShapeType::Circle)
        {
            const auto& circle =
                static_cast<const Circle&>(*this->shape);

            const float r = circle.GetRadius();

            inertia = 0.5f * this->mass * r * r;
        }
        else if (this->shape->GetType() == ShapeType::Box)
        {
            const auto& box = static_cast<const Box&>(*this->shape);

            const float w = box.GetWidth();
            const float h = box.GetHeight();

            inertia = this->mass * (w * w + h * h) / 12.0f;
        }
    }
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

    if(inertia > 0.0001f){

        const float angularAcceleration = torqueAccum/inertia;

        angularVelocity += angularAcceleration * dt;
    }

    angle += angularVelocity * dt;

    force = Vec2{0.0f, 0.0f};
    torqueAccum = 0.0f;
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

void RigidBody::SetAngularVelocity(float av)
{
    angularVelocity = av;
}

float RigidBody::GetAngle() const
{
    return angle;
}

float RigidBody::GetAngularVelocity() const
{
    return angularVelocity;
}

void RigidBody::ApplyTorque(float torque)
{
    torqueAccum += torque;
}

float RigidBody::GetInertia() const
{
    return inertia;
}

void RigidBody::ApplyForceAtPoint(
    const Vec2& appliedForce,
    const Vec2& worldPoint)
{

    ApplyForce(appliedForce);


    const Vec2 r = worldPoint - position;

    
    const float torque = r.x * appliedForce.y - r.y * appliedForce.x;

    ApplyTorque(torque);
}

void RigidBody::GroundCollision(float groundY)
{
    if (!shape || mass <= 0.0f)
        return;

    if (shape->GetType() == ShapeType::Circle)
    {
        const auto& circle = static_cast<const Circle&>(*shape);
        const float r = circle.GetRadius();

        const float bottom = position.y + r;
        const float penetration = bottom - groundY;

        if (penetration <= 0.0f)
            return;

        position.y -= penetration;

        if (velocity.y <= 0.0f)
            return;

        const float invMass = 1.0f / mass;
        const float e = (std::abs(velocity.y) < 15.0f) ? 0.0f : std::clamp(restitution, 0.0f, 1.0f);
        const float impulseMagnitude = (1.0f + e) * velocity.y / invMass;

        velocity.y -= impulseMagnitude * invMass;
    }
    else if (shape->GetType() == ShapeType::Box)
    {
        const auto& box = static_cast<const Box&>(*shape);
        const float hw = box.GetWidth() * 0.5f;
        const float hh = box.GetHeight() * 0.5f;

        const float c = std::cos(angle);
        const float s = std::sin(angle);

        const Vec2 corners[4] = {
            Vec2(-hw, -hh),
            Vec2( hw, -hh),
            Vec2( hw,  hh),
            Vec2(-hw,  hh)
        };

        struct ContactPoint {
            Vec2 r;
            float penetration;
        };
        std::vector<ContactPoint> contacts;

        float maxPenetration = 0.0f;

        for (const Vec2& corner : corners)
        {
            const Vec2 rotated(
                corner.x * c - corner.y * s,
                corner.x * s + corner.y * c
            );

            const float bottom = position.y + rotated.y;
            const float penetration = bottom - groundY;

            if (penetration > 0.0f)
            {
                contacts.push_back({ rotated, penetration });
                if (penetration > maxPenetration)
                {
                    maxPenetration = penetration;
                }
            }
        }

        if (contacts.empty())
            return;

        position.y -= maxPenetration;

        const float invMass = 1.0f / mass;
        const float invInertia = inertia > 0.0001f ? 1.0f / inertia : 0.0f;
        const float numContacts = static_cast<float>(contacts.size());

        for (const auto& cp : contacts)
        {
            const float contactVelocityY = velocity.y + angularVelocity * cp.r.x;

            if (contactVelocityY <= 0.0f)
                continue;

            const float crossRN = -cp.r.x;
            const float denominator = (invMass / numContacts) + crossRN * crossRN * invInertia;

            if (denominator <= 0.0f)
                continue;

            const float e = (std::abs(contactVelocityY) < 15.0f) ? 0.0f : std::clamp(restitution, 0.0f, 1.0f);
            const float impulseMag = (1.0f + e) * contactVelocityY / denominator;

            velocity.y -= (impulseMag * invMass) / numContacts;
            angularVelocity += crossRN * impulseMag * invInertia;
        }

        if (contacts.size() >= 2 && std::abs(velocity.y) < 20.0f)
        {
            angularVelocity *= 0.9f;
            if (std::abs(angularVelocity) < 0.01f)
                angularVelocity = 0.0f;
        }
    }
}

bool RigidBody::isGrounded(float groundY)
{
    
    if (!shape)
        return false;

    float halfExtentY = 0.0f;

    if (shape->GetType() == ShapeType::Circle)
    {
        const auto& circle = static_cast<const Circle&>(*shape);

        halfExtentY = circle.GetRadius();
    }
    else if (shape->GetType() == ShapeType::Box)
    {
        const auto& box = static_cast<const Box&>(*shape);

        const float halfWidth = box.GetWidth() * 0.5f;
        const float halfHeight = box.GetHeight() * 0.5f;

        halfExtentY = halfWidth * std::abs(std::sin(angle)) + halfHeight * std::abs(std::cos(angle));
    }

    const float bottom = position.y + halfExtentY;
    
    return bottom >= groundY - 0.5f;
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
        normal = Vec2(1.0f, 0.0f);
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

    // Contact point offsets
    const Vec2 rA = normal * GetRadius();
    const Vec2 rB = normal * (-other.GetRadius());

    const Vec2 vA = velocity + Vec2(-angularVelocity * rA.y, angularVelocity * rA.x);
    const Vec2 vB = other.velocity + Vec2(-other.angularVelocity * rB.y, other.angularVelocity * rB.x);

    const Vec2 relativeVelocity = vB - vA;

    const float velocityAlongNormal =
        relativeVelocity.x * normal.x +
        relativeVelocity.y * normal.y;

    if (velocityAlongNormal >= 0.0f)
        return;

    const float uA = rA.x * normal.y - rA.y * normal.x;
    const float uB = rB.x * normal.y - rB.y * normal.x;

    const float invInertiaA = inertia > 0.0001f ? 1.0f / inertia : 0.0f;
    const float invInertiaB = other.inertia > 0.0001f ? 1.0f / other.inertia : 0.0f;

    const float denominator = invMassSum + (uA * uA * invInertiaA) + (uB * uB * invInertiaB);
    if (denominator <= 0.0f)
        return;

    const float r =
        std::min(restitution, other.restitution);

    const float impulseMagnitude =
        -(1.0f + r) * velocityAlongNormal
        / denominator;

    const Vec2 impulse = normal * impulseMagnitude;

    velocity -= impulse * invMassA;
    other.velocity += impulse * invMassB;

    angularVelocity -= (uA * impulseMagnitude) * invInertiaA;
    other.angularVelocity += (uB * impulseMagnitude) * invInertiaB;
}

void RigidBody::ResolveCircleBox(RigidBody& boxBody){
    if (shape->GetType() != ShapeType::Circle ||
        boxBody.shape->GetType() != ShapeType::Box)
    {
        return;
    }

    const auto& circle = static_cast<const Circle&>(*shape);
    const auto& box    = static_cast<const Box&>(*boxBody.shape);

    const float rad        = circle.GetRadius();
    const float halfWidth  = box.GetWidth()  * 0.5f;
    const float halfHeight = box.GetHeight() * 0.5f;

    // --- Transform circle centre into the box's local (rotated) frame ---
    const Vec2 delta = position - boxBody.position;

    const float cosB = std::cos(boxBody.angle);
    const float sinB = std::sin(boxBody.angle);

    // Rotate by -angle (inverse rotation)
    const Vec2 localCircle(
         delta.x * cosB + delta.y * sinB,
        -delta.x * sinB + delta.y * cosB
    );

    // Closest point on the AABB in local space
    const float closestX = std::clamp(localCircle.x, -halfWidth,  halfWidth);
    const float closestY = std::clamp(localCircle.y, -halfHeight, halfHeight);

    // Normal from circle toward closest point (local space)
    Vec2 localNormal(
        closestX - localCircle.x,
        closestY - localCircle.y
    );

    float distSq = localNormal.x * localNormal.x + localNormal.y * localNormal.y;

    if (distSq >= rad * rad)
        return;

    float distance = std::sqrt(distSq);

    if (distance < 0.0001f)
    {
        // Circle centre is inside the box – push along shortest axis
        const float left   = localCircle.x + halfWidth;
        const float right  = halfWidth  - localCircle.x;
        const float top    = localCircle.y + halfHeight;
        const float bottom = halfHeight - localCircle.y;

        const float nearest = std::min(
            std::min(left, right),
            std::min(top, bottom)
        );

        if (nearest == left)
            localNormal = Vec2(-1.0f,  0.0f);
        else if (nearest == right)
            localNormal = Vec2( 1.0f,  0.0f);
        else if (nearest == top)
            localNormal = Vec2( 0.0f, -1.0f);
        else
            localNormal = Vec2( 0.0f,  1.0f);

        distance = 0.0f;
    }
    else
    {
        localNormal = localNormal / distance;
    }

    // --- Rotate normal back to world space ---
    const Vec2 normal(
        localNormal.x * cosB - localNormal.y * sinB,
        localNormal.x * sinB + localNormal.y * cosB
    );

    // Closest point on box in world space
    const Vec2 rB(
        closestX * cosB - closestY * sinB,
        closestX * sinB + closestY * cosB
    );
    const Vec2 contactPoint = boxBody.position + rB;
    const Vec2 rA = contactPoint - position;

    // --- Position correction ---
    const float invMassA   = 1.0f / mass;
    const float invMassB   = 1.0f / boxBody.mass;
    const float invMassSum = invMassA + invMassB;

    const float penetration = rad - distance;
    const float correctionMagnitude =
        std::max(penetration - 0.01f, 0.0f) * 0.8f / invMassSum;

    const Vec2 correction = normal * correctionMagnitude;

    position         -= correction * invMassA;
    boxBody.position += correction * invMassB;

    // --- Velocity resolution including angular effects ---
    const Vec2 vA = velocity + Vec2(-angularVelocity * rA.y, angularVelocity * rA.x);
    const Vec2 vB = boxBody.velocity + Vec2(-boxBody.angularVelocity * rB.y, boxBody.angularVelocity * rB.x);

    const Vec2 relativeVelocity = vB - vA;

    const float velocityAlongNormal =
        relativeVelocity.x * normal.x +
        relativeVelocity.y * normal.y;

    if (velocityAlongNormal >= 0.0f)
        return;

    const float uA = rA.x * normal.y - rA.y * normal.x;
    const float uB = rB.x * normal.y - rB.y * normal.x;

    const float invInertiaA = inertia > 0.0001f ? 1.0f / inertia : 0.0f;
    const float invInertiaB = boxBody.inertia > 0.0001f ? 1.0f / boxBody.inertia : 0.0f;

    const float denominator = invMassSum + (uA * uA * invInertiaA) + (uB * uB * invInertiaB);
    if (denominator <= 0.0f)
        return;

    const float resti = std::min(restitution, boxBody.restitution);

    const float impulseMagnitude =
        -(1.0f + resti) * velocityAlongNormal / denominator;

    const Vec2 impulse = normal * impulseMagnitude;

    velocity         -= impulse * invMassA;
    boxBody.velocity += impulse * invMassB;

    angularVelocity         -= (uA * impulseMagnitude) * invInertiaA;
    boxBody.angularVelocity += (uB * impulseMagnitude) * invInertiaB;
}

 
void RigidBody::ResolveBoxBox(RigidBody& other){

    if (shape->GetType()       != ShapeType::Box ||
        other.shape->GetType() != ShapeType::Box)
    {
        return;
    }

    const auto& boxA = static_cast<const Box&>(*shape);
    const auto& boxB = static_cast<const Box&>(*other.shape);

    const float halfWA = boxA.GetWidth()  * 0.5f;
    const float halfHA = boxA.GetHeight() * 0.5f;
    const float halfWB = boxB.GetWidth()  * 0.5f;
    const float halfHB = boxB.GetHeight() * 0.5f;

    // Rotation info for each box
    const float cosA = std::cos(angle);
    const float sinA = std::sin(angle);
    const float cosB = std::cos(other.angle);
    const float sinB = std::sin(other.angle);

    // Local axes (unit vectors) in world space
    const Vec2 axesA[2] = {
        Vec2( cosA, sinA),   // local X-axis of A
        Vec2(-sinA, cosA)    // local Y-axis of A
    };
    const Vec2 axesB[2] = {
        Vec2( cosB, sinB),   // local X-axis of B
        Vec2(-sinB, cosB)    // local Y-axis of B
    };

    const float halfExtentsA[2] = { halfWA, halfHA };
    const float halfExtentsB[2] = { halfWB, halfHB };

    const Vec2 delta = other.position - position;

    float minOverlap = std::numeric_limits<float>::max();
    Vec2  bestAxis(0.0f, 0.0f);

    // --- SAT: test the 4 separating axes (2 per box) ---

    // Axes from box A
    for (int i = 0; i < 2; i++)
    {
        const Vec2& axis = axesA[i];

        // A projected onto its own axis = its half-extent
        const float projA = halfExtentsA[i];

        // B projected onto this axis
        const float projB =
            halfExtentsB[0] * std::abs(axesB[0].x * axis.x + axesB[0].y * axis.y)
          + halfExtentsB[1] * std::abs(axesB[1].x * axis.x + axesB[1].y * axis.y);

        const float dist = std::abs(delta.x * axis.x + delta.y * axis.y);

        const float overlap = projA + projB - dist;
        if (overlap <= 0.0f) return;   // separating axis found

        if (overlap < minOverlap)
        {
            minOverlap = overlap;
            bestAxis   = axis;
        }
    }

    // Axes from box B
    for (int i = 0; i < 2; i++)
    {
        const Vec2& axis = axesB[i];

        const float projA =
            halfExtentsA[0] * std::abs(axesA[0].x * axis.x + axesA[0].y * axis.y)
          + halfExtentsA[1] * std::abs(axesA[1].x * axis.x + axesA[1].y * axis.y);

        const float projB = halfExtentsB[i];

        const float dist = std::abs(delta.x * axis.x + delta.y * axis.y);

        const float overlap = projA + projB - dist;
        if (overlap <= 0.0f) return;

        if (overlap < minOverlap)
        {
            minOverlap = overlap;
            bestAxis   = axis;
        }
    }

    // Ensure normal points from A toward B
    const float d = delta.x * bestAxis.x + delta.y * bestAxis.y;
    const Vec2 normal = d < 0.0f
        ? Vec2(-bestAxis.x, -bestAxis.y)
        : bestAxis;

    // --- Find contact point(s) ---
    const Vec2 localCornersA[4] = {
        Vec2(-halfWA, -halfHA),
        Vec2( halfWA, -halfHA),
        Vec2( halfWA,  halfHA),
        Vec2(-halfWA,  halfHA)
    };
    Vec2 worldCornersA[4];
    for (int i = 0; i < 4; i++) {
        worldCornersA[i] = position + Vec2(
            localCornersA[i].x * cosA - localCornersA[i].y * sinA,
            localCornersA[i].x * sinA + localCornersA[i].y * cosA
        );
    }

    const Vec2 localCornersB[4] = {
        Vec2(-halfWB, -halfHB),
        Vec2( halfWB, -halfHB),
        Vec2( halfWB,  halfHB),
        Vec2(-halfWB,  halfHB)
    };
    Vec2 worldCornersB[4];
    for (int i = 0; i < 4; i++) {
        worldCornersB[i] = other.position + Vec2(
            localCornersB[i].x * cosB - localCornersB[i].y * sinB,
            localCornersB[i].x * sinB + localCornersB[i].y * cosB
        );
    }

    std::vector<Vec2> contactPoints;
    const float eps = minOverlap + 0.5f;

    // Check corners of A inside B
    for (int i = 0; i < 4; i++) {
        const Vec2 rel = worldCornersA[i] - other.position;
        const float lx =  rel.x * cosB + rel.y * sinB;
        const float ly = -rel.x * sinB + rel.y * cosB;
        if (std::abs(lx) <= halfWB + eps && std::abs(ly) <= halfHB + eps) {
            contactPoints.push_back(worldCornersA[i]);
        }
    }

    // Check corners of B inside A
    for (int i = 0; i < 4; i++) {
        const Vec2 rel = worldCornersB[i] - position;
        const float lx =  rel.x * cosA + rel.y * sinA;
        const float ly = -rel.x * sinA + rel.y * cosA;
        if (std::abs(lx) <= halfWA + eps && std::abs(ly) <= halfHA + eps) {
            contactPoints.push_back(worldCornersB[i]);
        }
    }

    Vec2 contactPoint(0.0f, 0.0f);
    if (!contactPoints.empty()) {
        for (const Vec2& cp : contactPoints) {
            contactPoint += cp;
        }
        contactPoint = contactPoint / static_cast<float>(contactPoints.size());
    } else {
        float maxProjA = -std::numeric_limits<float>::infinity();
        Vec2 deepestA = worldCornersA[0];
        for (int i = 0; i < 4; i++) {
            float p = worldCornersA[i].x * normal.x + worldCornersA[i].y * normal.y;
            if (p > maxProjA) {
                maxProjA = p;
                deepestA = worldCornersA[i];
            }
        }

        float minProjB = std::numeric_limits<float>::infinity();
        Vec2 deepestB = worldCornersB[0];
        for (int i = 0; i < 4; i++) {
            float p = worldCornersB[i].x * normal.x + worldCornersB[i].y * normal.y;
            if (p < minProjB) {
                minProjB = p;
                deepestB = worldCornersB[i];
            }
        }
        contactPoint = (deepestA + deepestB) * 0.5f;
    }

    // --- Position correction ---
    const float invMassA   = 1.0f / mass;
    const float invMassB   = 1.0f / other.mass;
    const float invMassSum = invMassA + invMassB;

    const float slop    = 0.01f;
    const float percent = 0.8f;

    const float correctionMag =
        percent * std::max(minOverlap - slop, 0.0f) / invMassSum;

    const Vec2 correction = normal * correctionMag;

    position       -= correction * invMassA;
    other.position += correction * invMassB;

    // --- Velocity resolution with angular velocity ---
    const Vec2 rA = contactPoint - position;
    const Vec2 rB = contactPoint - other.position;

    const Vec2 vA = velocity + Vec2(-angularVelocity * rA.y, angularVelocity * rA.x);
    const Vec2 vB = other.velocity + Vec2(-other.angularVelocity * rB.y, other.angularVelocity * rB.x);

    const Vec2 relativeVelocity = vB - vA;

    const float velocityAlongNormal =
        relativeVelocity.x * normal.x +
        relativeVelocity.y * normal.y;

    if (velocityAlongNormal >= 0.0f) return;

    const float uA = rA.x * normal.y - rA.y * normal.x;
    const float uB = rB.x * normal.y - rB.y * normal.x;

    const float invInertiaA = inertia > 0.0001f ? 1.0f / inertia : 0.0f;
    const float invInertiaB = other.inertia > 0.0001f ? 1.0f / other.inertia : 0.0f;

    const float denominator = invMassSum + (uA * uA * invInertiaA) + (uB * uB * invInertiaB);
    if (denominator <= 0.0f) return;

    const float r = std::min(restitution, other.restitution);

    const float impulseMag =
        -(1.0f + r) * velocityAlongNormal / denominator;

    const Vec2 impulse = normal * impulseMag;

    velocity       -= impulse * invMassA;
    other.velocity += impulse * invMassB;

    angularVelocity       -= (uA * impulseMag) * invInertiaA;
    other.angularVelocity += (uB * impulseMag) * invInertiaB;
}
