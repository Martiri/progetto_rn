#ifndef BOID_HPP
#define BOID_HPP

#include <vector>

namespace boids_sim {

struct Vector2D {
  float x;
  float y;

  Vector2D operator+(const Vector2D &other) const;
  Vector2D operator-(const Vector2D &other) const;
  Vector2D operator*(float scalar) const;
  Vector2D &operator+=(const Vector2D &other);
  float norm2() const;
};
class boid {
 private:
  Vector2D position_{0.f, 0.f};
  Vector2D velocity_{0.f, 0.f};

 public:
  boid() = default;
  boid(float x, float y, float vx, float vy);

  const Vector2D &getPosition() const;
  const Vector2D &getVelocity() const;
  void updateposition(float dt, float maxX, float maxY);
  void updatevelocity(const Vector2D &vel, float vmax2);
};
}  // namespace boids_sim
#endif
