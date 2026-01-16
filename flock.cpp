#include "flock.hpp"

#include <cassert>
#include <random>
#include <stdexcept>

#include "SimValues.hpp"
#include "Vector2D.hpp"
#include "boid.hpp"
#include "predator.hpp"

namespace boids_sim {

flock::flock(const SimValues &sim_values)
    : predator_(sim_values.maxX / 2, sim_values.maxY / 2, 1.f, 1.f),
      numBoids_(sim_values.numBoids),
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
const predator &flock::getPredator() const { return predator_; }
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
    /* if (cx < 0) cx = 0;
    if (cy < 0) cy = 0;

    // 2. Evita indici troppo grandi (Overflow) - PREVIENE CRASH FUTURI
    if (cx >= sim_values.factorx) cx = sim_values.factorx - 1;
    if (cy >= sim_values.factory) cy = sim_values.factory - 1; */
    int cell = cy * sim_values.factorx + cx;
    next_[i] = headers_[cell];
    headers_[cell] = i;
  }
  for (int i = 0; i < numBoids_; ++i) {
    // Vector2D cumpos{0.f, 0.f};
    Vector2D cumvel{0.f, 0.f};
    Vector2D cumdist{0.f, 0.f};
    Vector2D cum_weighted_shortdist{0.f, 0.f};
    // Vector2D velocity = boids_[i].getVelocity();
    // float velocity_norm2 = velocity.norm2();
    int l{0};
    // int cell = getcell(boids_[i].getPosition(), factorx, d);
    int cx = getXcoord(boids_[i].getPosition(), sim_values.d);
    int cy = getYcoord(boids_[i].getPosition(), sim_values.d);
    /*if (cx >= sim_values.factorx) cx = sim_values.factorx - 1;
    if (cy >= sim_values.factory) cy = sim_values.factory - 1;
    if (cx < 0) cx = 0;
    if (cy < 0) cy = 0; */
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
          if (dist2 < sim_values.d2 /*&&
               velocity.cos_bigger_than_neg(sim_values.max_cos2_of_view, dist,
                                           velocity_norm2, dist2)*/) {
            l++;
            // cumpos += hposition;
            cumdist += dist;
            cumvel += hvelocity;
            if (dist2 < sim_values.ds2 && dist2 != 0)
              cum_weighted_shortdist += dist * (1.f / dist2);
          }
          b = next_[b];
        }
      }
    Vector2D acce{0.f, 0.f};
    Vector2D predator_dist =
        b_toroidaldistance(boids_[i].getPosition(), predator_);
    float predator_dist_norm2 = predator_dist.norm2();
    if (predator_dist_norm2 < sim_values.escape_d2) {
      Vector2D e_desired_velocity{0.f, 0.f};
      if (predator_dist_norm2 > 0.1f)
        e_desired_velocity = predator_dist.scale_to(sim_values.vmax) *
                             std::sqrt(sim_values.escape_d2 /
                                       std::max(predator_dist_norm2, 1.f));
      acce = boids_[i].tuneacceleration(e_desired_velocity, sim_values.accmax) *
             sim_values.e;
    }
    Vector2D accs{0.f, 0.f};
    Vector2D acca{0.f, 0.f};
    Vector2D accc{0.f, 0.f};
    if (l != 0) {
      float inv_l = 1.f / l;
      Vector2D s_desired_velocity{0.f, 0.f};
      if (cum_weighted_shortdist.norm2() > 0)
        s_desired_velocity = cum_weighted_shortdist.scale_to(sim_values.vmax);
      Vector2D unweighted_accs =
          boids_[i].tuneacceleration(s_desired_velocity, sim_values.accmax);
      Vector2D neighbours_AV = cumvel * inv_l;
      Vector2D a_desired_velocity{0.f, 0.f};
      if (neighbours_AV.norm2() > 0)
        a_desired_velocity = neighbours_AV.scale_to(sim_values.vmax);
      Vector2D unweighted_acca =
          boids_[i].tuneacceleration(a_desired_velocity, sim_values.accmax);
      Vector2D neighbours_MC_dist = cumdist * inv_l;
      Vector2D c_desired_velocity{0.f, 0.f};
      if (neighbours_MC_dist.norm2() > 0)
        c_desired_velocity = neighbours_MC_dist.scale_to(sim_values.vmax);
      Vector2D unweighted_accc =
          boids_[i].tuneacceleration(c_desired_velocity, sim_values.accmax);

      accs = unweighted_accs * (-sim_values.s);
      acca = unweighted_acca * sim_values.a;
      accc = unweighted_accc * sim_values.c;
    }
    newaccelerations_[i] = accs + acca + accc + acce;
  }
  // predator update
  Vector2D cumdist{0.f, 0.f};
  int l{0};
  for (const boid &b : boids_) {
    Vector2D dist = b_toroidaldistance(b.getPosition(), predator_);
    if (dist.norm2() < sim_values.predator_d2) {
      cumdist += dist;
      l++;
    }
  }
  if (l != 0) {
    Vector2D visible_MC_dist = cumdist * (1.f / l);
    predator_.updatechasingacceleration(
        visible_MC_dist, sim_values.predator_vmax, sim_values.predator_accmax,
        sim_values.ch);
    predator_.updatevelocity(sim_values.dt, sim_values.predator_vmax);
    predator_.updateposition(sim_values.dt, sim_values.maxX, sim_values.maxY);
  }
  predator_.resetacceleration();

  for (int i = 0; i < numBoids_; i++) {
    boid &b = boids_[i];
    // b.updaterandombehaviour(sim_values);
    b.updateacceleration(newaccelerations_[i]);
    b.updatevelocity(sim_values.dt, sim_values.vmax);
    b.updateposition(sim_values.dt, sim_values.maxX, sim_values.maxY);
    // boids_[i].updatefatigue(fatigue_threshold_v2_);
    // boids_[i].updaterush(rush_threshold_v2_, comeback_threshold_v2_);
  }
}
};  // namespace boids_sim
