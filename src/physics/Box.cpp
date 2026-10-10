#include "Box.h"

Box::Box(float width, float height)
    : width(width > 0.0f ? width : 1.0f),
      height(height > 0.0f ? height : 1.0f)
{
}

ShapeType Box::GetType() const
{
    return ShapeType::Box;
}

float Box::GetWidth() const
{
    return width;
}

float Box::GetHeight() const
{
    return height;
}

void Box::SetSize(float width, float height)
{
    if (width > 0.0f && height > 0.0f)
    {
        width = width;
        height = height;
    }
}