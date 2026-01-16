#ifndef FLOCK_HPP
#define FLOCK_HPP

#include <cmath>
#include <vector>

#include "Vector2D.hpp"
#include "SimValues.hpp"
#include "boid.hpp"
#include "predator.hpp"

namespace boids_sim {
class flock {
 private:
  std::vector<boid> boids_;
  predator predator_;
  int numBoids_;
  // togliere?
  float maxX_;
  float maxY_;
  //
  std::vector<int> headers_;
  std::vector<int> next_;
  std::vector<Vector2D> newaccelerations_;

 public:
  flock(const SimValues& sim_values);
  int getcell(const Vector2D& position, const int factorx, const float d) const;
  int getXcoord(const Vector2D& position, const float d) const;
  int getYcoord(const Vector2D& position, const float d) const;
  void step(const SimValues& sim_values);
  const std::vector<boid>& getBoids() const;
  const predator& getPredator() const;
  void reset_headers() {}
  Vector2D computeAverageVelocity() const;
  float computeAverageDistance() const;
  Vector2D b_toroidaldistance(const Vector2D& hpos, const boid& b) const;
};
};  // namespace boids_sim

#endif