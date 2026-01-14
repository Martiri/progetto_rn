#ifndef VECTOR2D_HPP
#define VECTOR2D_HPP

namespace boids_sim {

struct Vector2D {
  float x;
  float y;

  Vector2D operator+(const Vector2D other) const;
  Vector2D operator-(const Vector2D other) const;
  Vector2D operator*(const float scalar) const;
  Vector2D &operator+=(const Vector2D other);
  Vector2D &operator*=(const float scalar);
  float norm2() const;
  // float dot(const Vector2D &other) const;
  bool cos_bigger_than_neg(const float max_cos2, const Vector2D other,
                           const float this_norm2,
                           const float other_norm2) const;
};
};  // namespace boids_sim

#endif