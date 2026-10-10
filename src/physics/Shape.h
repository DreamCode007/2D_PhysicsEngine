#pragma once

enum class ShapeType
{
    Circle,
    Box,
    Polygon
};

class Shape
{
public:
    virtual ~Shape() = default;

    virtual ShapeType GetType() const = 0;
};