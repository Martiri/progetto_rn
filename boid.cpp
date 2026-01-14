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
void boid::updateacceleration(const Vector2D acc, const SimValues& sim_values) {
  const float acc_norm2 = acc.norm2();
  if (acc_norm2 <= sim_values.accmax2)
    acceleration_ = acc;
  else
    acceleration_ = acc * (1.f / std::sqrt(acc_norm2)) * sim_values.accmax;
  acceleration_ += random_acceleration_;
}
void boid::updatevelocity(const SimValues& sim_values) {
  Vector2D v_new = velocity_ + acceleration_ * sim_values.dt;
  const float v_new_norm2 = v_new.norm2();
  if (v_new_norm2 <= sim_values.vmax2)
    velocity_ = v_new;
  else
    velocity_ = v_new * (1.f / std::sqrt(v_new_norm2)) * sim_values.vmax;
}
void boid::updateposition(const SimValues& sim_values) {
  position_ += velocity_ * sim_values.dt;
  if (position_.x < 0) {
    position_.x += sim_values.maxX;
  } else if (position_.x > sim_values.maxX) {
    position_.x -= sim_values.maxX;
  }
  if (position_.y < 0) {
    position_.y += sim_values.maxY;
  } else if (position_.y > sim_values.maxY) {
    position_.y -= sim_values.maxY;
  }
}
void boid::updaterandombehaviour(const SimValues& sim_values) {
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
}
};  // namespace boids_sim