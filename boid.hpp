#ifndef BOID_HPP
#define BOID_HPP

#include <vector>

#include "Vector2D.hpp"
#include "SimValues.hpp"

namespace boids_sim {
class boid {
 private:
  Vector2D position_{0.f, 0.f};
  Vector2D velocity_{0.f, 0.f};
  Vector2D acceleration_{0.f, 0.f};
  Vector2D random_acceleration_{0.f, 0.f};
  Vector2D rythm_acceleration_{0.f, 0.f};
  int random_timer_{0};
  int fatigue_{0};
  int rush_{0};

  // int rush_{0};
  // int fatigue_{0};

 public:
  boid() = default;
  boid(const float x, const float y, const float vx, const float vy);

  const Vector2D &getPosition() const;
  const Vector2D &getVelocity() const;
  const Vector2D &getAcceleration() const;
  // void velocityalignmentacceleration(const float idealv2);
  void updateposition(const SimValues& sim_values);
  void updatevelocity(const SimValues& sim_values);
  void updateacceleration(const Vector2D acc, const SimValues& sim_values);
  void updaterandombehaviour(const SimValues& sim_values);
  void updatefatigue(const float fatigue_threshold_v2)
  void updaterush(const float rush_threshold_v2, const float comeback_threshold_v2)
  void updaterythmstate(const SimValues& sim_values)
  void updatefatigueacceleration(const SimValues& sim_values);
  void updaterushacceleration(const SimValues& sim_values);
  void updaterythmacceleration(const SimValues& sim_values);
};
};  // namespace boids_sim

#endif