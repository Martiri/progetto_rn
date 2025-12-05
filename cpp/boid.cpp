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
    : position_{x, y}, velocity_{vx, vy} {}
const Vector2D &boid::getPosition() const { return position_; }
const Vector2D &boid::getVelocity() const { return velocity_; }
void boid::updatevelocity(const Vector2D &vel) { velocity_ += vel; }
void boid::updateposition(float dt) { position_ += velocity_ * dt; };

}  // namespace boids_sim