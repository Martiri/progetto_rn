#include "flock.hpp"

#include <SFML/Graphics.hpp>
#include <algorithm>
#include <cassert>
#include <execution>
#include <random>
#include <stdexcept>

#include "Vector2D.hpp"
#include "boid.hpp"
#include "predator.hpp"
#include "simvalues.hpp"

namespace boids_sim {

flock::flock(const SimValues &sv)
    : predator_(sv.maxX / 2, sv.maxY / 2, 1.f, 1.f), boids_num_(sv.boids_num) {
  boids_.clear();
  boids_.reserve(static_cast<size_t>(boids_num_));

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> distX(
      sv.maxX * sv.spawn_inf_edgeX_coeff, sv.maxX * sv.spawn_sup_edgeX_coeff);
  std::uniform_real_distribution<float> distY(
      sv.maxY * sv.spawn_inf_edgeY_coeff, sv.maxY * sv.spawn_sup_edgeY_coeff);
  std::uniform_real_distribution<float> distV(
      -sv.distV_amplitude + sv.distV_offset,
      sv.distV_amplitude + sv.distV_offset);

  for (int i = 0; i < boids_num_; ++i) {
    boids_.emplace_back(distX(gen), distY(gen), distV(gen), distV(gen));
  }
  headers_.resize(sv.ncells, -1);
  next_.resize(boids_num_, -1);
  newaccelerations_.resize(boids_num_);
 /* boids_vertices_.resize(boids_num_ * 3);
  boids_vertices_.setPrimitiveType(sf::Triangles);
  predator_shape_.setFillColor(sf::Color::Red);
  predator_shape_.setPointCount(3);
  predator_shape_.setPoint(0, sf::Vector2f(9.9f, 0.f));
  predator_shape_.setPoint(1, sf::Vector2f(-9.9f, -6.6f));
  predator_shape_.setPoint(2, sf::Vector2f(-9.9f, 6.6f));
  predator_shape_.setOrigin(0.f, 0.f); */
}
const std::vector<boid> &flock::getBoids() const { return boids_; }
int flock::get_boids_num() const { return boids_num_; }
const predator &flock::getPredator() const { return predator_; }
void flock::reset_headers() { std::fill(headers_.begin(), headers_.end(), -1); }
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
void flock::sort_boids_vector(const float factorx, const float d) {
  std::sort(boids_.begin(), boids_.end(), [&](const boid &b1, const boid &b2) {
    return getcell(b1.getPosition(), factorx, d) <
           getcell(b2.getPosition(), factorx, d);
  });
}
int flock::get_boid_index(const boid &b) { return &b - &boids_[0]; }
void flock::populate_grid(const float d, const float factorx) {
  std::for_each(boids_.begin(), boids_.end(), [&](const boid &b) {
    size_t i = get_boid_index(b);
    float cy = getYcoord(b.getPosition(), d);
    float cx = getXcoord(b.getPosition(), d);
    int cell = cy * factorx + cx;
    next_[i] = headers_[cell];
    headers_[cell] = i;
  });
}
void flock::step(const SimValues &sv) {
  // grid&boids_ preparation
  // if (sv.steps_counter % 180 == 0) sort_boids_vector(sv.factorx, sv.d);
  reset_headers();
  populate_grid(sv.d, sv.factorx);
  // collection of information
  std::for_each(
      std::execution::par, boids_.begin(), boids_.end(),
      [&](const boid &current_boid) {
        size_t current_boid_idx = &current_boid - &boids_[0];
        Vector2D cumvel{0.f, 0.f};
        Vector2D cumdist{0.f, 0.f};
        Vector2D cum_weighted_shortdist{0.f, 0.f};
        int act_cell = 0;
        int neighbours_count{0};
        int cx = getXcoord(current_boid.getPosition(), sv.d);
        int cy = getYcoord(current_boid.getPosition(), sv.d);
        for (int j = -1; j <= 1; j++)
          for (int k = -1; k <= 1; k++) {
            act_cell = ((cx + k + sv.factorx) % sv.factorx) +
                       ((cy + j + sv.factory) % sv.factory) * sv.factorx;
            int b = headers_[act_cell];
            while (b != -1) {
              current_boid.collect_infos(
                  boids_[b].getPosition(), boids_[b].getVelocity(), sv.maxX,
                  sv.maxY, sv.d2, sv.ds2, neighbours_count, cumdist, cumvel,
                  cum_weighted_shortdist);
              b = next_[b];
            }
          }
        // update of boids' acceleration
        Vector2D acce = current_boid.calculate_escaping_acceleration(
            predator_.getPosition(), sv.e, sv.escape_d2, sv.vmax, sv.accmax,
            sv.maxX, sv.maxY);
        Vector2D accs{0.f, 0.f};
        Vector2D acca{0.f, 0.f};
        Vector2D accc{0.f, 0.f};
        if (neighbours_count != 0) {
          float inv_neighbours_count = 1.f / neighbours_count;
          accs = current_boid.calculate_separation_acceleration(
              cum_weighted_shortdist, sv.s, sv.vmax, sv.accmax);
          acca = current_boid.calculate_alignment_acceleration(
              cumvel, inv_neighbours_count, sv.a, sv.vmax, sv.accmax);
          accc = current_boid.calculate_cohesion_acceleration(
              cumdist, inv_neighbours_count, sv.c, sv.vmax, sv.accmax);
        }
        newaccelerations_[current_boid_idx] = accs + acca + accc + acce;
      });
  //
  Vector2D cumdist{0.f, 0.f};
  int neighbours_count{0};
  std::for_each(boids_.begin(), boids_.end(), [&](const boid &b) {
    predator_.collect_predator_infos(b.getPosition(), sv.predator_d2, sv.maxX,
                                     sv.maxY, cumdist, neighbours_count);
  });
  if (neighbours_count != 0) {
    Vector2D visible_MC_dist = cumdist * (1.f / neighbours_count);
    predator_.updatechasingacceleration(visible_MC_dist, sv.predator_vmax,
                                        sv.predator_accmax, sv.ch);
    predator_.updatevelocity(sv.dt, sv.predator_vmax);
    predator_.updateposition(sv.dt, sv.maxX, sv.maxY);
  }
  predator_.resetacceleration();

  std::for_each(std::execution::par, boids_.begin(), boids_.end(),
                [&](boid &b) {
                  size_t i = &b - &boids_[0];
                  boids_[i].updateacceleration(newaccelerations_[i]);
                  boids_[i].updatevelocity(sv.dt, sv.vmax);
                  boids_[i].updateposition(sv.dt, sv.maxX, sv.maxY);
                });
}
/*
void flock::update_vertices() {
  std::for_each(std::execution::par, boids_.begin(), boids_.end(),
                [this](const boid &boid) {
                  size_t i = &boid - &boids_[0];
                  size_t vtx_i = i * 3;
                  Vector2D pos = boid.getPosition();
                  Vector2D vel = boid.getVelocity();
                  float vel_norm = std::sqrt(vel.norm2());
                  float inv_vel_norm;
                  if (vel_norm != 0)
                    inv_vel_norm = 1.f / vel_norm;
                  else
                    return;
                  float sin = vel.y * inv_vel_norm;
                  float cos = vel.x * inv_vel_norm;
                  float size = boid_size_;
                  boids_vertices_[vtx_i].position =
                      sf::Vector2f(pos.x + cos * size, pos.y + sin * size);
                  // Retro-sinistra
                  boids_vertices_[vtx_i + 1].position =
                      sf::Vector2f(pos.x + (-5.1f * cos + 3.4f * sin),
                                   pos.y + (-5.1f * sin - 3.4f * cos));
                  // Retro-destra
                  boids_vertices_[vtx_i + 2].position =
                      sf::Vector2f(pos.x + (-5.1f * cos - 3.4f * sin),
                                   pos.y + (-5.1f * sin + 3.4f * cos));
                });
  Vector2D pos = predator_.getPosition();
  Vector2D vel = predator_.getVelocity();
  predator_shape_.setPosition(pos.x, vel.y);
  predator_shape_.setRotation(std::atan2(vel.y, vel.x) * 57.2958f);
} */
};  // namespace boids_sim
