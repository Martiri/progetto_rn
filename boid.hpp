#ifndef BOID_HPP
#define BOID_HPP

#include <vector>

extern float brndf;
extern int b_cost_rnd_time_magnitude;
extern int b_var_rnd_time_magnitude;
extern float velafp;
extern float velafm;
extern float fatiguef;
extern float rushf;
namespace boids_sim {
struct Vector2D {
  float x;
  float y;

  Vector2D operator+(const Vector2D &other) const;
  Vector2D operator-(const Vector2D &other) const;
  Vector2D operator*(float scalar) const;
  Vector2D &operator+=(const Vector2D &other);
  Vector2D &operator*=(const float scalar);
  float norm2() const;
  float dot(const Vector2D &other) const;
};
class boid {
 private:
  Vector2D position_{0.f, 0.f};
  Vector2D velocity_{0.f, 0.f};
  Vector2D acceleration_{0.f, 0.f};
  Vector2D random_acceleration_{0.f, 0.f};

  int random_timer_{0};
  int rush_{0};
  int fatigue_{0};

 public:
  boid() = default;
  boid(float x, float y, float vx, float vy, float ax, float ay, float rnd_accx,
       float rnd_accy, int rnd_timer);

  const Vector2D &getPosition() const;
  const Vector2D &getVelocity() const;
  const Vector2D &getAcceleration() const;
  // void velocityalignmentacceleration(float idealv2);
  void updateposition(float dt, float maxX, float maxY);
  void updatevelocity(float dt, float vmax, float vmax2);
  void updateacceleration(const Vector2D &acc, float accmax, float accmax2,
                          const Vector2D &swerve_acc,
                          const Vector2D &fatigue_acc,
                          const Vector2D &rush_acc);
  void updaterandombehaviour(float accmax);
  Vector2D updateswerveacceleration();
  Vector2D updatefatigueacceleration(const float stamina,
                                     const float fatigue_threshold_v2) const;
  Vector2D updaterushacceleration(const float patience,
                                  const float rush_threshold_v2) const;
  void updatefatigue(float fatigue_threshold_v2);
  void updaterush(float rush_threshold_v2, float comeback_threshold_v2);
};
}  // namespace boids_sim

#endif
