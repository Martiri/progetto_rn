#include "predator.hpp"

namespace boids_sim {

void predator::collect_predator_infos(const Vector2D boid_position,
                                      const SimValues& sv,
                                      CumulativeInfos& cum_infos) const {
  Vector2D dist = boid_position.toroidal_minus(getPosition(), sv.maxX, sv.maxY);
  if (dist.norm2() < sv.predator_d2) {
    cum_infos.cumdist += dist;
    cum_infos.neighbors_count++;
  }
};
Vector2D predator::calculate_chasing_acceleration(
    const Vector2D visible_MC_dist, const float predator_vmax,
    const float predator_accmax, const float ch) {
  Vector2D desired_velocity{visible_MC_dist.scale_to(predator_vmax)};
  return tune_acceleration(desired_velocity, predator_accmax) * ch;
}
};  // namespace boids_sim