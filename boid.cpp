#include "boid.hpp"

#include <cmath>
#include <vector>

float brndf = 0.45;
int b_cost_rnd_time_magnitude = 1200;
int b_var_rnd_time_magnitude = b_cost_rnd_time_magnitude + 1;
float velafp = 0.6f;
float velafm = 0.95f;
float fatiguef = -0.6f;
float rushf = 0.5f;

namespace boids_sim {
Vector2D Vector2D::operator+(const Vector2D &other) const {
  return {x + other.x, y + other.y};
}
Vector2D Vector2D::operator-(const Vector2D &other) const {
  return {x - other.x, y - other.y};
}
Vector2D Vector2D::operator*(float scalar) const {
  return {x * scalar, y * scalar};
}
Vector2D &Vector2D::operator+=(const Vector2D &other) {
  x += other.x;
  y += other.y;
  return *this;
}
Vector2D &Vector2D::operator*=(const float scalar) {
  x *= scalar;
  y *= scalar;
  return *this;
}

float Vector2D::norm2() const { return x * x + y * y; }
float Vector2D::dot(const Vector2D &other) const {
  return x * other.x + y * other.y;
}
boid::boid(float x, float y, float vx, float vy, float ax, float ay,
           float rnd_accx, float rnd_accy, int rnd_timer)
    : position_{x, y},
      velocity_{vx, vy},
      acceleration_{ax, ay},
      random_acceleration_{rnd_accx, rnd_accy},
      random_timer_{rnd_timer},
      rush_{0},
      fatigue_{0} {};
const Vector2D &boid::getPosition() const { return position_; }
const Vector2D &boid::getVelocity() const { return velocity_; }
const Vector2D &boid::getAcceleration() const { return acceleration_; }
void boid::updaterandombehaviour(float accmax) {
  if (random_timer_ <= 0) {
    float acc_magnitude = brndf * accmax;
    random_acceleration_ = {
        (static_cast<float>(rand()) / RAND_MAX * 2.f - 1.f) * acc_magnitude,
        (static_cast<float>(rand()) / RAND_MAX * 2.f - 1.f) * acc_magnitude};
    random_timer_ =
        b_cost_rnd_time_magnitude + rand() % b_var_rnd_time_magnitude;
  }
  random_timer_--;
}
Vector2D boid::updateswerveacceleration() {
  if (velocity_.dot(acceleration_) <
      0.5 * std::sqrt(acceleration_.norm2()) *
          std::sqrt(velocity_.norm2()))  // || velocity_.norm2() < idealv2) {
    return acceleration_ * velafp;
  else
    return {0.f, 0.f};
}
Vector2D boid::updatefatigueacceleration(
    const float stamina, const float fatigue_threshold_v2) const {
  if (fatigue_ >= stamina && velocity_.norm2() > fatigue_threshold_v2)
    return velocity_ * fatiguef;
  else
    return {0.f, 0.f};
}
Vector2D boid::updaterushacceleration(const float patience,
                                      const float comeback_threshold_v2) const {
  if (rush_ >= patience && velocity_.norm2() < comeback_threshold_v2)
    return velocity_ * rushf;
  else
    return {0.f, 0.f};
}
void boid::updateacceleration(const Vector2D &acc, float accmax, float accmax2,
                              const Vector2D &swerve_acc,
                              const Vector2D &fatigue_acc,
                              const Vector2D &rush_acc) {
  float acc_norm2 = acc.norm2();
  if (acc_norm2 <= accmax2) {
    acceleration_ = acc;
  } else {
    acceleration_ = acc * (1.f / std::sqrt(acc_norm2)) * accmax;
  }
  acceleration_ += random_acceleration_;
  acceleration_ += swerve_acc;
  acceleration_ += fatigue_acc;
  acceleration_ += rush_acc;
}
void boid::updatevelocity(float dt, float vmax, float vmax2) {
  Vector2D v_new = velocity_ + acceleration_ * dt;
  float v_new_norm2 = v_new.norm2();
  if (v_new_norm2 <= vmax2) {
    velocity_ = v_new;
  } else {
    velocity_ = v_new * (1.f / std::sqrt(v_new_norm2)) * vmax;
  }
}
void boid::updateposition(float dt, float maxX, float maxY) {
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
void boid::updatefatigue(float fatigue_threshold_v2) {
  if (velocity_.norm2() > fatigue_threshold_v2)
    fatigue_++;
  else if (fatigue_ > 0)
    fatigue_--;
}
void boid::updaterush(float rush_threshold_v2, float comeback_threshold_v2) {
  if (velocity_.norm2() < rush_threshold_v2)
    rush_++;
  else if (velocity_.norm2() > comeback_threshold_v2 && rush_ > 0)
    rush_--;
}
};  // namespace boids_sim