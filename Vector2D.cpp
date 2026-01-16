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

float Vector2D::norm2() const { return x * x + y * y; }
/* float Vector2D::dot(const Vector2D &other) const {
  return x * other.x + y * other.y;
} */
bool Vector2D::cos_bigger_than_neg(const float max_cos2, const Vector2D other,
                                   const float this_norm2,
                                   const float other_norm2) const {
  float dot = x * other.x + y * other.y;
  if (dot >= 0)
    return true;
  else {
    float dot2 = dot * dot;
    float min_unacc_dot2 = max_cos2 * this_norm2 * other_norm2;
    if (dot2 < min_unacc_dot2)
      return true;
    else
      return false;
  }
}
Vector2D Vector2D::scale_to(const float wanted_norm) const {
  // float scaling_factor = 0.f;
  // if (norm2() > 0) 
  float scaling_factor = (1.f / std::sqrt(norm2())) * wanted_norm;
  return *this * scaling_factor;
}
};  // namespace boids_sim