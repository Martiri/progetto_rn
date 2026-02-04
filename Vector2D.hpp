#ifndef VECTOR2D_HPP
#define VECTOR2D_HPP

namespace boids_sim {

struct Vector2D {
  float x;
  float y;

  Vector2D operator+(const Vector2D other) const;
  Vector2D operator-(const Vector2D other) const;
  Vector2D operator*(const float scalar) const;
  Vector2D operator/(const float scalar) const;
  Vector2D &operator+=(const Vector2D other);
  Vector2D &operator*=(const float scalar);
  Vector2D &operator/=(const float scalar);
  Vector2D toroidal_minus(const Vector2D other, const float maxX,
                                    const float maxY) const;
  float norm2() const;
  float norm() const;
  Vector2D scale_to(const float wanted_norm) const;
};
};  // namespace boids_sim

#endif