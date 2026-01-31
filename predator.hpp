#ifndef PREDATOR_HPP
#define PREDATOR_HPP

#include "Vector2D.hpp"
#include "boid.hpp"
#include "simvalues.hpp"

namespace boids_sim {
class predator : public boid {
 public:
  using boid::boid;
  void collect_predator_infos(const Vector2D& boid_position,
                              const float predator_d2, const float maxX,
                              const float maxY, Vector2D& cumdist,
                              int& neighbours_count) const;
  void updatechasingacceleration(const Vector2D visible_MC,
                                 const float predator_vmax,
                                 const float predator_accmax, const float ch);
  void resetacceleration();
};
};  // namespace boids_sim

#endif