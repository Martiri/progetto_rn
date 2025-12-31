#ifndef FLOCK_HPP
#define FLOCK_HPP

#include <vector>
#include <cmath>
#include "boid.hpp"

extern float hamplitudev;
extern float offsetv;
extern float fact;
extern float inf_spawnfX;
extern float sup_spawnfX;
extern float inf_spawnfY;
extern float sup_spawnfY;
extern float s;
extern float a;
extern float c;
extern float d;
extern float d2;
extern float ds;
extern float ds2;
extern float timescale;
extern float vmax;
extern float idealv;
extern float staminaf;
extern float lazinessf;
extern float comebackf;
extern float fatigue_threshold_v;
extern float rush_threshold_v;
extern float comeback_threshold_v;
extern int stamina;
extern int patience;

namespace boids_sim {
class flock {
 private:
  std::vector<boid> boids_;
  int numBoids_;
  float maxX_;
  float maxY_;
  std::vector<int> headers_;
  std::vector<int> next_;
  std::vector<Vector2D> newaccelerations_;
  float idealv2_;
  float fatigue_threshold_v2_;
  float rush_threshold_v2_;
  float comeback_threshold_v2_;
  float stamina_;
  float patience_;

 public:
  flock(int numBoids, float maxX, float maxY, int ncells, float idealv,
        float fatigue_threshold_v, float rush_threshold_v,
        float comeback_threshold_v);
  int getcell(const Vector2D &position, const int &factorx,
              const float &length) const;
  int getXcoord(const Vector2D &position, const float &length) const;
  int getYcoord(const Vector2D &position, const float &length) const;
  void step(float dt, float maxX, float maxY, int factorx, int factory, float s,
            float a, float c, float d2, float ds2, float length, float vmax,
            float vmax2, float accmax, float accmax2);
  const std::vector<boid> &getBoids() const;
  void reset_headers() {}
  Vector2D computeAverageVelocity() const;
  float computeAverageDistance() const;
  Vector2D b_toroidaldistance(const Vector2D &hpos, const boid &b) const;
};
};  // namespace boids_sim#endif
#endif