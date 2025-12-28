#ifndef FLOCK_HPP
#define FLOCK_HPP

#include <vector>

#include "boid.hpp"


namespace boids_sim {
class flock {
 private:
  std::vector<boid> boids_;
  int numBoids_;
  float maxX_;
  float maxY_;
  std::vector<int> headers_;
  std::vector<int> next_;
  std::vector<Vector2D> newvelocity_;

 public:
  flock(int numBoids, float maxX, float maxY, int ncells);
  int getcell(const Vector2D &position, const int &factorx,
              const float &length) const;
  void step(float dt, float maxX, float maxY, int factorx, float s, float a,
            float c, float d2, float ds2, float length, int ncells, float vmax2);
  const std::vector<boid> &getBoids() const;
  void reset_headers() {}
  Vector2D computeAverageVelocity() const;
  float computeAverageDistance() const;
};
}; 
#endif
