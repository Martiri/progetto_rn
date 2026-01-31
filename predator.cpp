#include "predator.hpp"

namespace boids_sim {

void predator::collect_predator_infos(const Vector2D& boid_position,
                            const float predator_d2, const float maxX,
                            const float maxY, Vector2D& cumdist,
                            int& neighbours_count) const {
  Vector2D dist = boid_position.toroidal_minus(getPosition(), maxX, maxY);
  if (dist.norm2() < predator_d2) {
    cumdist += dist;
    neighbours_count++;
  }
};
void predator::updatechasingacceleration(const Vector2D visible_MC_dist,
                                         const float predator_vmax,
                                         const float predator_accmax,
                                         const float ch) {
  Vector2D desired_velocity{0.f, 0.f};
  if (visible_MC_dist.norm2() > 0)
    desired_velocity = visible_MC_dist.scale_to(predator_vmax);
  acceleration_ = tuneacceleration(desired_velocity, predator_accmax) * ch;
}
void predator::resetacceleration() { acceleration_ = {0.f, 0.f}; }

};  // namespace boids_sim