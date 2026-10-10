#pragma once

#include "Shape.h"

class Box : public Shape
{
private:
    float width;
    float height;

public:
    Box(float width, float height);

    ShapeType GetType() const override;

    float GetWidth() const;
    float GetHeight() const;

    void SetSize(float width, float height);
};