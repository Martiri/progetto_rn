#include "flock.hpp"

#include <cassert>
#include <random>
#include <stdexcept>


float hamplitudev = 200.f;
float offsetv = 70.f;
float fact = 0.5f;
float inf_spawnfX = 0.5f - fact * (1.f / (2 * std::sqrt(2)));
float sup_spawnfX = 0.5f + fact * (1.f / (2 * std::sqrt(2)));
float inf_spawnfY = inf_spawnfX;
float sup_spawnfY = sup_spawnfX;
float s;
float a;
float c;
float d;
float d2 = d * d;
float ds;
float ds2 = ds * ds;
float timescale;
float vmax = 250.f;
float idealv = vmax * 0.5;
float staminaf = 0.5f;
float lazinessf = 0.3f;
float comebackf = 0.8f;
float fatigue_threshold_v = vmax * staminaf;
float rush_threshold_v = vmax * lazinessf;
float comeback_threshold_v = vmax * comebackf;
int stamina = 200;
int patience = 50;

namespace boids_sim {

flock::flock(int numBoids, float maxX, float maxY, int ncells, float idealv,
             float fatigue_threshold_v, float rush_threshold_v,
             float comeback_threshold_v)
    : numBoids_(numBoids),
      maxX_(maxX),
      maxY_(maxY),
      idealv2_(idealv * idealv),
      fatigue_threshold_v2_(fatigue_threshold_v * fatigue_threshold_v),
      rush_threshold_v2_(rush_threshold_v * rush_threshold_v),
      comeback_threshold_v2_(comeback_threshold_v * comeback_threshold_v),
      stamina_(stamina),
      patience_(patience) {
  boids_.clear();
  boids_.reserve(static_cast<size_t>(numBoids_));

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> distX(maxX_ * inf_spawnfX,
                                              maxX_ * sup_spawnfX);
  std::uniform_real_distribution<float> distY(maxY_ * inf_spawnfY,
                                              maxY_ * sup_spawnfY);
  std::uniform_real_distribution<float> distV(-hamplitudev + offsetv,
                                              hamplitudev + offsetv);

  for (int i = 0; i < numBoids_; ++i) {
    boids_.emplace_back(distX(gen), distY(gen), distV(gen), distV(gen), 0.f,
                        0.f, 0.f, 0.f, 0);
  }
  headers_.resize(ncells, -1);
  next_.resize(numBoids_, -1);
  newaccelerations_.resize(numBoids_);
}
const std::vector<boid> &flock::getBoids() const { return boids_; }
Vector2D flock::b_toroidaldistance(const Vector2D &hpos, const boid &b) const {
  Vector2D dist = hpos - b.getPosition();
  if (dist.x > maxX_ * 0.5f)
    dist.x -= maxX_;
  else if (dist.x < -maxX_ * 0.5f)
    dist.x += maxX_;
  if (dist.y > maxY_ * 0.5f)
    dist.y -= maxY_;
  else if (dist.y < -maxY_ * 0.5f)
    dist.y += maxY_;
  return dist;
}
int flock::getcell(const Vector2D &position, const int &factorx,
                   const float &length) const {
  return static_cast<int>(position.y / length) * factorx +
         static_cast<int>(position.x / length);
}
int flock::getXcoord(const Vector2D &position, const float &length) const {
  return static_cast<int>(position.x / length);
}
int flock::getYcoord(const Vector2D &position, const float &length) const {
  return static_cast<int>(position.y / length);
}
void flock::step(float dt, float maxX, float maxY, int factorx, int factory,
                 float s, float a, float c, float d2, float ds2, float length,
                 float vmax, float vmax2, float accmax, float accmax2) {
  std::fill(headers_.begin(), headers_.end(), -1);
  for (int i = 0; i < numBoids_; ++i) {
    // int cell = getcell(boids_[i].getPosition(), factorx, length);
    int cx = getXcoord(boids_[i].getPosition(), length);
    int cy = getYcoord(boids_[i].getPosition(), length);
    int cell = cy * factorx + cx;
    next_[i] = headers_[cell];
    headers_[cell] = i;
  }
  for (int i = 0; i < numBoids_; ++i) {
    // Vector2D cumpos{0.f, 0.f};
    Vector2D cumvel{0.f, 0.f};
    Vector2D cumdist{0.f, 0.f};
    Vector2D cumshortdist{0.f, 0.f};
    int l{0};
    // int cell = getcell(boids_[i].getPosition(), factorx, length);
    int cx = getXcoord(boids_[i].getPosition(), length);
    int cy = getYcoord(boids_[i].getPosition(), length);
    for (int j = -1; j <= 1; j++)
      for (int k = -1; k <= 1; k++) {
        int act_cell{((cx + k + factorx) % factorx) +
                     ((cy + j + factory) % factory) * factorx};
        // if (act_cell >= 0 && act_cell < ncells) {
        int b = headers_[act_cell];
        while (b != -1) {
          const Vector2D &hposition = boids_[b].getPosition();
          const Vector2D &hvelocity = boids_[b].getVelocity();
          // Vector2D dist = hposition - boids_[i].getPosition();
          Vector2D dist = b_toroidaldistance(hposition, boids_[i]);
          float dist2 = dist.norm2();
          Vector2D velocity = boids_[i].getVelocity();
          if (dist2 < d2 &&
              velocity.dot(dist) >
                  -0.5f * std::sqrt(velocity.norm2()) * std::sqrt(dist2)) {
            l++;
            // cumpos += hposition;
            cumdist += dist;
            cumvel += hvelocity;
            if (dist2 < ds2) cumshortdist += dist;
          }
          b = next_[b];
        }
        // }
      }
    if (l != 0) {
      Vector2D accs = cumshortdist * (-s);
      Vector2D acca = (cumvel * (1.f / l) - boids_[i].getVelocity()) * a;
      // Vector2D accc = (cumpos * (1.f / l) - boids_[i].getPosition()) * c;
      Vector2D accc = cumdist * (1.f / l) * c;
      newaccelerations_[i] = accs + acca + accc;
    }
  }
  for (int i = 0; i < numBoids_; i++) {
    boids_[i].updaterandombehaviour(accmax);
    boids_[i].updateacceleration(
        newaccelerations_[i], accmax, accmax2,
        boids_[i].updateswerveacceleration(),
        boids_[i].updatefatigueacceleration(stamina_, fatigue_threshold_v2_),
        boids_[i].updaterushacceleration(patience_, rush_threshold_v2_));
    boids_[i].updatevelocity(dt, vmax, vmax2);
    boids_[i].updateposition(dt, maxX, maxY);
    boids_[i].updatefatigue(fatigue_threshold_v2_);
    boids_[i].updaterush(rush_threshold_v2_, comeback_threshold_v2_);
  }
}
};  // namespace boids_sim