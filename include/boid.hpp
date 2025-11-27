#ifndef BOID_HPP
#define BOID_HPP

#include <vector>

namespace boids_sim
{

    struct Vector2D
    {
        double x;
        double y;

        Vector2D operator+(const Vector2D &other) const;
        Vector2D operator-(const Vector2D &other) const;
        Vector2D operator*(double scalar) const;
        Vector2D &operator+=(const Vector2D &other);
        double norm2() const;
    };
    class boid
    {
    private:
        Vector2D position_{0.0, 0.0};
        Vector2D velocity_{0.0, 0.0};
        

    public:
        boid() = default;
        boid(double x, double y, double vx, double vy);

        const Vector2D &getPosition() const;
        const Vector2D &getVelocity() const;

        void updatepositon(double dt, double maxX, double maxY);
        void updatevelocity();
    };
}
#endif