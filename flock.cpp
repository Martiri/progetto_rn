#include "flock.hpp"

#include <cassert>
#include <random>
#include <stdexcept>

#include "SimValues.hpp"
#include "Vector2D.hpp"
#include "boid.hpp"

namespace boids_sim {

flock::flock(const SimValues &sim_values)
    : numBoids_(sim_values.numBoids),
      maxX_(sim_values.maxX),
      maxY_(sim_values.maxY) {
  boids_.clear();
  boids_.reserve(static_cast<size_t>(numBoids_));

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> distX(
      maxX_ * sim_values.spawn_inf_edgeX_coeff,
      maxX_ * sim_values.spawn_sup_edgeX_coeff);
  std::uniform_real_distribution<float> distY(
      maxY_ * sim_values.spawn_inf_edgeY_coeff,
      maxY_ * sim_values.spawn_sup_edgeY_coeff);
  std::uniform_real_distribution<float> distV(
      -sim_values.distV_amplitude + sim_values.distV_offset,
      sim_values.distV_amplitude + sim_values.distV_offset);

  for (int i = 0; i < numBoids_; ++i) {
    boids_.emplace_back(distX(gen), distY(gen), distV(gen), distV(gen));
  }
  headers_.resize(sim_values.ncells, -1);
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
int flock::getXcoord(const Vector2D &position, const float d) const {
  return static_cast<int>(position.x / d);
}
int flock::getYcoord(const Vector2D &position, const float d) const {
  return static_cast<int>(position.y / d);
}
int flock::getcell(const Vector2D &position, const int factorx,
                   const float d) const {
  return getYcoord(position, d) * factorx + getXcoord(position, d);
}
void flock::step(const SimValues &sim_values) {
  std::fill(headers_.begin(), headers_.end(), -1);
  for (int i = 0; i < numBoids_; ++i) {
    // int cell = getcell(boids_[i].getPosition(), factorx, d);
    float cy = getYcoord(boids_[i].getPosition(), sim_values.d);
    float cx = getXcoord(boids_[i].getPosition(), sim_values.d);
    int cell = cy * sim_values.factorx + cx;
    next_[i] = headers_[cell];
    headers_[cell] = i;
  }
  for (int i = 0; i < numBoids_; ++i) {
    // Vector2D cumpos{0.f, 0.f};
    Vector2D cumvel{0.f, 0.f};
    Vector2D cumdist{0.f, 0.f};
    Vector2D cumshortdist{0.f, 0.f};
    Vector2D velocity = boids_[i].getVelocity();
    float velocity_norm2 = velocity.norm2();
    int l{0};
    // int cell = getcell(boids_[i].getPosition(), factorx, d);
    int cx = getXcoord(boids_[i].getPosition(), sim_values.d);
    int cy = getYcoord(boids_[i].getPosition(), sim_values.d);
    for (int j = -1; j <= 1; j++)
      for (int k = -1; k <= 1; k++) {
        int act_cell{((cx + k + sim_values.factorx) % sim_values.factorx) +
                     ((cy + j + sim_values.factory) % sim_values.factory) *
                         sim_values.factorx};
        // if (act_cell >= 0 && act_cell < ncells) {
        int b = headers_[act_cell];
        while (b != -1) {
          const Vector2D &hposition = boids_[b].getPosition();
          const Vector2D &hvelocity = boids_[b].getVelocity();
          // Vector2D dist = hposition - boids_[i].getPosition();
          Vector2D dist = b_toroidaldistance(hposition, boids_[i]);
          float dist2 = dist.norm2();
          if (dist2 < sim_values.d2 &&
              velocity.cos_bigger_than_neg(sim_values.max_cos2_of_view, dist,
                                           velocity_norm2, dist2)) {
            l++;
            // cumpos += hposition;
            cumdist += dist;
            cumvel += hvelocity;
            if (dist2 < sim_values.ds2) cumshortdist += dist;
          }
          b = next_[b];
        }
        // }
      }
    Vector2D accs{0.f, 0.f};
    Vector2D acca{0.f, 0.f};
    Vector2D accc{0.f, 0.f};
    if (l != 0) {
      float inv_l = 1.f / l;
      accs = cumshortdist * (-sim_values.s);
      acca = (cumvel * inv_l - boids_[i].getVelocity()) * sim_values.a;
      // Vector2D accc = (cumpos * (1.f / l) - boids_[i].getPosition()) *
      // c;
      accc = cumdist * inv_l * sim_values.c;
    }
    newaccelerations_[i] = accs + acca + accc;
  }
  for (int i = 0; i < numBoids_; i++) {
    boid &b = boids_[i];
    b.updaterandombehaviour(sim_values);
    b.updateacceleration(newaccelerations_[i], sim_values);
    b.updatevelocity(sim_values);
    b.updateposition(sim_values);
    // boids_[i].updatefatigue(fatigue_threshold_v2_);
    // boids_[i].updaterush(rush_threshold_v2_, comeback_threshold_v2_);
  }
}
};  // namespace boids_sim
