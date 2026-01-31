#ifndef BOID_HPP
#define BOID_HPP

#include <vector>

#include "Vector2D.hpp"
#include "simvalues.hpp"

namespace boids_sim {
class boid {
 protected:
  Vector2D position_{0.f, 0.f};
  Vector2D velocity_{0.f, 0.f};
  Vector2D acceleration_{0.f, 0.f};
  Vector2D random_acceleration_{0.f, 0.f};
  int random_timer_{0};

 public:
  boid() = default;
  virtual ~boid() = default;
  boid(const float x, const float y, const float vx, const float vy);

  const Vector2D& getPosition() const;
  const Vector2D& getVelocity() const;
  const Vector2D& getAcceleration() const;

  void collect_infos(const Vector2D& other_position,
                     const Vector2D& other_velocity, const float maxX,
                     const float maxY, const float d2, const float ds2,
                     int& neighbours_count, Vector2D& cumdist, Vector2D& cumvel,
                     Vector2D& cum_weighted_shortdist) const;
  Vector2D tuneacceleration(const Vector2D desired_velocity,
                            const float accmax) const;
  Vector2D calculate_escaping_acceleration(const Vector2D predator_position,
                                           const float e, const float escape_d2,
                                           const float vmax, const float accmax,
                                           const float maxX,
                                           const float maxY) const;
  Vector2D calculate_separation_acceleration(
      const Vector2D cum_weighted_shortdist, const float s, const float vmax,
      const float accmax) const;
  Vector2D calculate_alignment_acceleration(const Vector2D cumvel,
                                            const float inv_neighbours_count,
                                            const float a, const float vmax,
                                            const float accmax) const;
  Vector2D calculate_cohesion_acceleration(const Vector2D cumdist,
                                           const float inv_neighbours_count,
                                           const float c, const float vmax,
                                           const float accmax) const;
  void updateposition(const float dt, const float maxX, const float maxY);
  void updatevelocity(const float dt, const float vmax);
  void updateacceleration(const Vector2D acc);
  void updaterandombehaviour(const SimValues& sv);
};
};  // namespace boids_sim

#endif