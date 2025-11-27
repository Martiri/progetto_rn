#include "flock.hpp"
#include <random>
float hamplitudev = 100.f;
float offsetv = 4.f;
namespace boids_sim
{

    flock::flock(int numBoids, double maxX, double maxY)
        : numBoids_(numBoids), maxX_(maxX), maxY_(maxY)
    {
        boids_.clear();
        boids_.reserve(static_cast<size_t>(numBoids_));

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> distX(0.0, maxX_);
        std::uniform_real_distribution<double> distY(0.0, maxY_);
        std::uniform_real_distribution<double> distV(-hamplitudev + offsetv, hamplitudev + offsetv);

        for (int i = 0; i < numBoids_; ++i)
        {
            boids_.emplace_back(distX(gen), distY(gen), distV(gen), distV(gen));
        }
        headers_.resize(ncells, -1);
        next_.resize(numBoids_, -1);
    }
    int flock::getcell(const Vector2D &position) const
    {
        return static_cast<int>(position.y / l) * factorx + static_cast<int>(position.x / l);
    }
    void flock::step()
    {
        for (int i = 0; i < numBoids_; ++i)
        {
            int cell = getcell(boids_[i].getPosition());
            if (headers_[cell] != -1)
            {
                next_[i] = headers_[cell];
            }

            headers_[cell] = i;
        }
        for (int i = 0; i < numBoids_; ++i)
        {
            int cell = getcell(boids_[i].getPosition());
            int a = headers_[cell];
        }
    }
}