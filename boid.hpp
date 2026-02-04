#ifndef BOID_HPP
#define BOID_HPP

#include "Vector2D.hpp"
#include "cumulativeinfos.hpp"
#include "simvalues.hpp"

namespace boids_sim {
class boid {
 protected:
  Vector2D position_{0.f, 0.f};
  Vector2D velocity_{0.f, 0.f};

 public:
  boid() = default;
  virtual ~boid() = default;
  boid(const float x, const float y, const float vx, const float vy);

  Vector2D getPosition() const;
  Vector2D getVelocity() const;

  void collect_infos(const Vector2D other_position,
                     const Vector2D other_velocity, const SimValues& sv,
                     CumulativeInfos& cum_infos) const;
  Vector2D tune_acceleration(const Vector2D desired_velocity,
                            const float accmax) const;
  Vector2D calculate_escaping_acceleration(const Vector2D predator_position,
                                           const SimValues& sv) const;
  Vector2D calculate_separation_acceleration(
      const Vector2D cum_weighted_shortdist, const float s, const float vmax,
      const float accmax) const;
  Vector2D calculate_alignment_acceleration(
      const Vector2D neighbors_average_velocity, const float a,
      const float vmax, const float accmax) const;
  Vector2D calculate_cohesion_acceleration(
      const Vector2D vector_to_center_of_mass, const float c, const float vmax,
      const float accmax) const;
  void update_position(const float dt, const float maxX, const float maxY);
  void update_velocity(const Vector2D acc, const float dt, const float vmax);
  void resettleX(const float old_maxX, const float new_maxX);
  void resettleY(const float old_maxY, const float new_maxX);
};
};  // namespace boids_sim

#endif