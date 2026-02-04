#include "Vector2D.hpp"
#include <cmath>

namespace boids_sim {
Vector2D Vector2D::operator+(const Vector2D other) const {
  return {x + other.x, y + other.y};
}
Vector2D Vector2D::operator-(const Vector2D other) const {
  return {x - other.x, y - other.y};
}
Vector2D Vector2D::operator*(const float scalar) const {
  return {x * scalar, y * scalar};
}
Vector2D Vector2D::operator/(const float scalar) const {
  return {x / scalar, y / scalar};
};
Vector2D &Vector2D::operator+=(const Vector2D other) {
  x += other.x;
  y += other.y;
  return *this;
}
Vector2D &Vector2D::operator*=(const float scalar) {
  x *= scalar;
  y *= scalar;
  return *this;
}
Vector2D &Vector2D::operator/=(const float scalar) {
  x /= scalar;
  y /= scalar;
  return *this;
}
Vector2D Vector2D::toroidal_minus(const Vector2D other, const float maxX,
                                  const float maxY) const {
  Vector2D dist = *this - other;
  if (dist.x > maxX * 0.5f)
    dist.x -= maxX;
  else if (dist.x < -maxX * 0.5f)
    dist.x += maxX;
  if (dist.y > maxY * 0.5f)
    dist.y -= maxY;
  else if (dist.y < -maxY * 0.5f)
    dist.y += maxY;
  return dist;
}
float Vector2D::norm2() const { return x * x + y * y; }
float Vector2D::norm() const { return std::sqrt(norm2()); }
Vector2D Vector2D::scale_to(const float wanted_norm) const {
  float _norm{norm()};
  if (_norm > 0.f) return *this / _norm * wanted_norm;
  return {0.f, 0.f};
}
};  // namespace boids_sim