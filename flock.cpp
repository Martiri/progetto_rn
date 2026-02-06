#include "flock.hpp"
#include <algorithm>
#include <execution>
#include <random>

namespace boids_sim {

flock::flock(FlockConfiguration &fc, const SimValues &sv) {
  fc.constrain(sv.maxX, sv.maxY, sv.vmax);
  grid_columns_num_ = static_cast<int>(sv.maxX / sv.d) + 1;
  grid_rows_num_ = static_cast<int>(sv.maxY / sv.d) + 1;
  const size_t boids_num = static_cast<size_t>(fc.boids_num);
  const size_t cells_num =
      static_cast<size_t>(grid_columns_num_ * grid_rows_num_);
  headers_.assign(cells_num, -1);
  next_.assign(boids_num, -1);
  new_accelerations_.assign(boids_num, {0.f, 0.f});
  boids_.reserve(boids_num);
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> distX(
      sv.maxX * fc.spawn_inf_edgeX_coeff, sv.maxX * fc.spawn_sup_edgeX_coeff);
  std::uniform_real_distribution<float> distY(
      sv.maxY * fc.spawn_inf_edgeY_coeff, sv.maxY * fc.spawn_sup_edgeY_coeff);
  std::uniform_real_distribution<float> distV(-fc.distV_amplitude,
                                              fc.distV_amplitude);
  for (size_t i = 0; i < boids_num; ++i) {
    boids_.emplace_back(distX(gen), distY(gen), distV(gen), distV(gen));
  }
  predator_ = {fc.predator_starting_position.x, fc.predator_starting_position.y,
               fc.predator_starting_velocity.x,
               fc.predator_starting_velocity.y};
}
const std::vector<boid> &flock::getBoids() const { return boids_; }
const predator &flock::getPredator() const { return predator_; }
size_t flock::get_boid_index(const boid &boid) const {
  return static_cast<size_t>(&boid - &boids_[0]);
}
void flock::reset_headers() { std::fill(headers_.begin(), headers_.end(), -1); }
int flock::getXcoord(const Vector2D position, const float d) const {
  if (d <= 0.f) {
    throw std::runtime_error("Cell size must be positive");
  }
  return static_cast<int>(position.x / d);
}
int flock::getYcoord(const Vector2D position, const float d) const {
  if (d <= 0.f) {
    throw std::runtime_error("Cell size must be positive");
  }
  return static_cast<int>(position.y / d);
}
int flock::getcell(const Vector2D position, const float d) const {
  return getYcoord(position, d) * grid_columns_num_ + getXcoord(position, d);
}
void flock::populate_grid(const float d, const float maxX, const float maxY) {
  reset_headers();
  std::for_each(boids_.begin(), boids_.end(),
                [this, d, maxX, maxY](const boid &boid) {
                  Vector2D boid_position = boid.getPosition();
                  size_t i{get_boid_index(boid)};
                  int cell{getcell(boid_position, d)};
                  next_[i] = headers_[static_cast<size_t>(cell)];
                  headers_[static_cast<size_t>(cell)] = static_cast<int>(i);
                });
}
void flock::compute_boids_accelerations(const SimValues &sv) {
  std::for_each(
      std::execution::par, boids_.begin(), boids_.end(),
      [this, &sv](const boid &current_boid) {
        size_t current_boid_idx{get_boid_index(current_boid)};
        CumulativeInfos current_boid_infos{};
        int current_cell{0};
        int cx{getXcoord(current_boid.getPosition(), sv.d)};
        int cy{getYcoord(current_boid.getPosition(), sv.d)};
        for (int j = -1; j <= 1; j++)
          for (int k = -1; k <= 1; k++) {
            current_cell = ((cx + k + grid_columns_num_) % grid_columns_num_) +
                           ((cy + j + grid_rows_num_) % grid_rows_num_) *
                               grid_columns_num_;
            int b{headers_[static_cast<size_t>(current_cell)]};
            while (b != -1) {
              current_boid.collect_infos(
                  boids_[static_cast<size_t>(b)].getPosition(),
                  boids_[static_cast<size_t>(b)].getVelocity(), sv,
                  current_boid_infos);
              b = static_cast<int>(next_[static_cast<size_t>(b)]);
            }
          }
        Vector2D e_acc{current_boid.calculate_escaping_acceleration(
            predator_.getPosition(), sv)};
        Vector2D s_acc{0.f, 0.f};
        Vector2D a_acc{0.f, 0.f};
        Vector2D c_acc{0.f, 0.f};
        if (current_boid_infos.neighbors_count > 0) {
          float inv_neighbors_count =
              1.f / static_cast<float>(current_boid_infos.neighbors_count);
          s_acc = current_boid.calculate_separation_acceleration(
              current_boid_infos.cum_weighted_shortdist, sv.s, sv.vmax,
              sv.accmax);
          Vector2D neighbors_avg_vel{current_boid_infos.cumvel *
                                     inv_neighbors_count};
          a_acc = current_boid.calculate_alignment_acceleration(
              neighbors_avg_vel, sv.a, sv.vmax, sv.accmax);
          Vector2D vector_to_mc{current_boid_infos.cumdist *
                                inv_neighbors_count};
          c_acc = current_boid.calculate_cohesion_acceleration(
              vector_to_mc, sv.c, sv.vmax, sv.accmax);
        }
        new_accelerations_[current_boid_idx] = s_acc + a_acc + c_acc + e_acc;
      });
}
void flock::predator_step(const SimValues &sv) {
  CumulativeInfos predator_infos{};
  std::for_each(boids_.begin(), boids_.end(),
                [this, &sv, &predator_infos](const boid &boid) {
                  predator_.collect_predator_infos(boid.getPosition(), sv,
                                                   predator_infos);
                });
  Vector2D ch_acc{0.f, 0.f};
  if (predator_infos.neighbors_count > 0) {
    Vector2D vector_to_visible_mc{
        predator_infos.cumdist /
        static_cast<float>(predator_infos.neighbors_count)};
    ch_acc = predator_.calculate_chasing_acceleration(
        vector_to_visible_mc, sv.predator_vmax, sv.predator_accmax, sv.ch);
  }
  predator_.update_velocity(ch_acc, sv.dt, sv.predator_vmax);
  predator_.update_position(sv.dt, sv.maxX, sv.maxY);
}
void flock::boids_step(const float vmax, const float dt, const float maxX,
                       const float maxY) {
  std::for_each(std::execution::par, boids_.begin(), boids_.end(),
                [this, vmax, dt, maxX, maxY](boid &boid) {
                  size_t i = get_boid_index(boid);
                  boid.update_velocity(new_accelerations_[i], dt, vmax);
                  boid.update_position(dt, maxX, maxY);
                });
}
void flock::step(const SimValues &sv) {
  populate_grid(sv.d, sv.maxX, sv.maxY);
  compute_boids_accelerations(sv);
  predator_step(sv);
  boids_step(sv.vmax, sv.dt, sv.maxX, sv.maxY);
}
};  // namespace boids_sim
