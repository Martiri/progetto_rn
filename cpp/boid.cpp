#include "boid.hpp"
#include <cmath>
#include <vector>

namespace boids_sim
{
    Vector2D Vector2D::operator+(const Vector2D &other) const
    {
        return {x + other.x, y + other.y};
    }
    Vector2D Vector2D::operator-(const Vector2D &other) const
    {
        return {x - other.x, y - other.y};
    }
    Vector2D Vector2D::operator*(double scalar) const
    {
        return {x * scalar, y * scalar};
    }
    Vector2D &Vector2D::operator+=(const Vector2D &other)
    {
        x += other.x;
        y += other.y;
        return *this;
    }
    double Vector2D::norm2() const
    {
        return x * x + y * y;
    }

}