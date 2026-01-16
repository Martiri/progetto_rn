#include "boid.hpp"

#include <cmath>
#include <iostream>
#include <vector>

#include "SimValues.hpp"
#include "Vector2D.hpp"

namespace boids_sim {
boid::boid(const float x, const float y, const float vx, const float vy)
    : position_{x, y}, velocity_{vx, vy}, acceleration_{0.f, 0.f} {}
const Vector2D& boid::getPosition() const { return position_; }
const Vector2D& boid::getVelocity() const { return velocity_; }
const Vector2D& boid::getAcceleration() const { return acceleration_; }
void boid::updateacceleration(
    const Vector2D acc /*, const SimValues& sim_values*/) {
  /* const float acc_norm2 = acc.norm2();
  if (acc_norm2 <= sim_values.accmax2) */
  acceleration_ = acc;
  // else
  //   acceleration_ = acc * (1.f / std::sqrt(acc_norm2)) * sim_values.accmax;
  // acceleration_ += random_acceleration_;
}
void boid::updatevelocity(const float dt, const float vmax) {
  velocity_+=acceleration_*dt;
  if (velocity_.norm2() > vmax * vmax) velocity_ = velocity_.scale_to(vmax);
}
void boid::updateposition(const float dt, const float maxX, const float maxY) {
  position_ += velocity_ * dt;
  if (position_.x < 0) {
    position_.x += maxX;
  } else if (position_.x > maxX) {
    position_.x -= maxX;
  }
  if (position_.y < 0) {
    position_.y += maxY;
  } else if (position_.y > maxY) {
    position_.y -= maxY;
  }
}
const Vector2D boid::tuneacceleration(const Vector2D desired_velocity,
                                       const float accmax) {
  Vector2D acceleration = desired_velocity - velocity_;
  if (acceleration.norm2() > accmax * accmax) acceleration = acceleration.scale_to(accmax);
  return acceleration;
}
/* void boid::updaterandombehaviour(const SimValues& sim_values) {
  if (random_timer_ <= 0) {
    float acc_magnitude =
        sim_values.random_behaviour_intensity_coeff * sim_values.accmax;
    random_acceleration_ = {
        (static_cast<float>(rand()) / RAND_MAX * 2.f - 1.f) * acc_magnitude,
        (static_cast<float>(rand()) / RAND_MAX * 2.f - 1.f) * acc_magnitude};
    random_timer_ = sim_values.const_random_behaviour_duration +
                    rand() % sim_values.var_random_behaviour_duration;
    // std::cout << "changed random!!!!!!" << std::endl;
  }
  random_timer_--;
} */
};  // namespace boids_sim