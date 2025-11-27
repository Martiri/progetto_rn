#ifndef FLOCK_HPP
#define FLOCK_HPP

#include <vector>

#include "boid.hpp"
#include "main.cpp"

float l{0.f};
namespace boids_sim {
class flock {
 private:
  std::vector<boid> boids_;
  int numBoids_;
  double maxX_;
  double maxY_;
  const std::vector<int> headers_;
  std::vector<int> next_;

 public:
 flock(int numBoids, double maxX, double maxY);
 
};

}  // namespace boids_sim
#endif