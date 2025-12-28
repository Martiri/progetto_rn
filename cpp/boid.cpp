#include "boid.hpp"

#include <cmath>
#include <vector>

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
float Vector2D::norm2() const { return x * x + y * y; }
boid::boid(float x, float y, float vx, float vy)
    : position_{x, y}, velocity_{vx, vy} {};
const Vector2D &boid::getPosition() const { return position_; }
const Vector2D &boid::getVelocity() const { return velocity_; }
void boid::updatevelocity(const Vector2D &vel, float vmax2) {
  Vector2D v_new = velocity_ + vel;
  if (v_new.norm2() <= vmax2) {
    velocity_ = v_new;
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

};  // namespace boids_sim
