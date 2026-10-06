#pragma once

#include <cmath>

class Vec2
{
public:
    float x;
    float y;

    Vec2()
    {
        x = 0;
        y = 0;
    }

    Vec2(float x, float y)
    {
        this->x = x;
        this->y = y;
    }

    Vec2 operator+(const Vec2& other) const
    {
        return Vec2(
            x + other.x,
            y + other.y
        );
    }

    Vec2 operator-(const Vec2& other) const
    {
        return Vec2(
            x - other.x,
            y - other.y
        );
    }

    Vec2 operator*(float scalar) const
    {
        return Vec2(
            x * scalar,
            y * scalar
        );
    }

    Vec2 operator/(float scalar) const
    {
        return Vec2(
            x / scalar,
            y / scalar
        );
    }

    float length() const
    {
        return std::sqrt(x * x + y * y);
    }

    Vec2 normalized() const
    {
        float len = length();

        if (len == 0)
            return Vec2(0, 0);

        return Vec2(x / len, y / len);
    }

    float dot(const Vec2& other) const
    {
        return x * other.x + y * other.y;
    }
};