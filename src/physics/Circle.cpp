#include "Circle.h"

Circle::Circle(float radius): radius(radius > 0.0f ? radius : 1.0f){}

ShapeType Circle::GetType() const
{

    return ShapeType::Circle;
}

float Circle::GetRadius() const
{
    return radius;
}

void Circle::SetRadius(float r){
    if(r > 0.0f){
        radius = r;
    }
}