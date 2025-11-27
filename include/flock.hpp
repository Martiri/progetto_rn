#ifndef FLOCK_HPP
#define FLOCK_HPP

#include "boid.hpp"
#include <vector>

float l{0.f};
int factorx = 20;
int factory = 20;
int ncells = factorx * factory;

namespace boids_sim
{
    class flock
    {
    private:
        std::vector<boid> boids_;
        int numBoids_;
        double maxX_;
        double maxY_;
        std::vector<int> headers_;
        std::vector<int> next_;

    public:
        flock(int numBoids, double maxX, double maxY);
        int getcell(const Vector2D& position) const;
        void step();
        const std::vector<boid> &getBoids() const;
        Vector2D computeAverageVelocity() const;
        double computeAverageDistance() const;
    };

}
#endif