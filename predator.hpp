#ifndef PREDATOR_HPP
#define PREDATOR_HPP

#include "boid.hpp"

namespace boids_sim {
class predator : public boid {
 public:
  using boid::boid;
  void collect_predator_infos(const Vector2D boid_position,
                              const SimValues& sv, CumulativeInfos& cum_infos) const;
  Vector2D calculate_chasing_acceleration(const Vector2D visible_MC,
                                 const float predator_vmax,
                                 const float predator_accmax, const float ch);
};
};  // namespace boids_sim

#endif