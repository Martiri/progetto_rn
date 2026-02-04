#ifndef FLOCK_HPP
#define FLOCK_HPP
#include "Vector2D.hpp"
#include <cstddef>
#include "flockconfiguration.hpp"
#include "predator.hpp"
#include "simvalues.hpp"

namespace boids_sim {
class flock {
 private:
  std::vector<boid> boids_;
  predator predator_;
  std::vector<int> headers_;
  std::vector<int> next_;
  std::vector<Vector2D> new_accelerations_;
  int grid_columns_num_;
  int grid_rows_num_;

 public:
  flock(FlockConfiguration& fc, const SimValues& sv);

  const std::vector<boid>& getBoids() const;
  const predator& getPredator() const;
  size_t get_boid_index(const boid& boid) const;
  int getXcoord(const Vector2D position, const float d) const;
  int getYcoord(const Vector2D position, const float d) const;
  int getcell(const Vector2D position, const float d) const;
  void reset_headers();
  void populate_grid(const float d, const float maxX, const float maxY);
  void compute_boids_accelerations(const SimValues& sv);
  void predator_step(const SimValues& sv);
  void boids_step(const float vmax, const float dt, const float maxX,
                  const float maxY);
  void step(const SimValues& sv);
  void update_grid(const float new_d, const float maxX, const float maxY);
  void resettleX(const float old_maxX, const float new_maxX);
  void resettleY(const float old_maxY, const float new_maxY);
  // Vector2D computeAverageVelocity() const;
  // float computeAverageDistance() const;
};
};  // namespace boids_sim

#endif