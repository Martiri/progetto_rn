#ifndef FLOCK_HPP
#define FLOCK_HPP

#include <cmath>
#include <vector>

#include "Vector2D.hpp"
#include "boid.hpp"
#include "predator.hpp"
#include "simvalues.hpp"

namespace boids_sim {
class flock {
 private:
  std::vector<boid> boids_;
  predator predator_;
  int boids_num_;
  float boid_size_;
  std::vector<int> headers_;
  std::vector<int> next_;
  std::vector<Vector2D> newaccelerations_;
  /* sf::VertexArray boids_vertices_;
  sf::ConvexShape predator_shape_; */
 public:
  flock(const SimValues& sv);
  int getcell(const Vector2D& position, const int factorx, const float d) const;
  void sort_boids_vector(const float factorx, const float d);
  int getXcoord(const Vector2D& position, const float d) const;
  int getYcoord(const Vector2D& position, const float d) const;
  int get_boid_index(const boid& b);
  void reset_headers();
  void populate_grid(const float d, const float factorx);
  void step(const SimValues& sv);
  void update_vertices();
  void set_boids_color();
  const std::vector<boid>& getBoids() const;
  int get_boids_num() const;
  const predator& getPredator() const;
  Vector2D computeAverageVelocity() const;
  float computeAverageDistance() const;
};
};  // namespace boids_sim

#endif