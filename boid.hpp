#ifndef BOID_HPP
#define BOID_HPP

#include <vector>

#include "Vector2D.hpp"
#include "SimValues.hpp"

namespace boids_sim {
class boid {
 protected:
  Vector2D position_{0.f, 0.f};
  Vector2D velocity_{0.f, 0.f};
  Vector2D acceleration_{0.f, 0.f};
  Vector2D random_acceleration_{0.f, 0.f};
  int random_timer_{0};

  // int rush_{0};
  // int fatigue_{0};

 public:
  boid() = default;
  boid(const float x, const float y, const float vx, const float vy);

  const Vector2D &getPosition() const;
  const Vector2D &getVelocity() const;
  const Vector2D &getAcceleration() const;

  const Vector2D tuneacceleration(const Vector2D desired_velocity, const float accmax);

  // void velocityalignmentacceleration(const float idealv2);
  void updateposition(const float dt, const float maxX, const float maxY);
  void updatevelocity(const float dt, const float vmax);
  void updateacceleration(const Vector2D acc/*, const SimValues& sim_values*/);
  void updaterandombehaviour(const SimValues& sim_values);
};
};  // namespace boids_sim

#endif