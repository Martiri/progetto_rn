#ifndef FLOCK_HPP
#define FLOCK_HPP

#include <vector>

#include "boid.hpp"

float l{0.f};
int factorx = 20;
int factory = 20;
int ncells = factorx * factory;

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
  flock(int numBoids, float maxX, float maxY);
  int getcell(const Vector2D& position) const;
  void step(float dt, float factorx, float s, float a, float c, float d,
            float ds);
  const std::vector<boid>& getBoids() const;
  Vector2D computeAverageVelocity() const;
  float computeAverageDistance() const;
};
};  // namespace boids_sim
#endif