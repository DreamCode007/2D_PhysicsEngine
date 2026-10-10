#pragma once

#include "Shape.h"

class Circle : public Shape
{
private:
    float radius;

public:
    Circle(float radius);

    ShapeType GetType() const override;

    float GetRadius() const;
    void SetRadius(float radius);
};