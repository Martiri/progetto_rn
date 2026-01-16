#ifndef PREDATOR_HPP
#define PREDATOR_HPP

#include "SimValues.hpp"
#include "Vector2D.hpp"
#include "boid.hpp"

namespace boids_sim {
class predator : public boid {
 public:
  using boid::boid;
  void updatechasingacceleration(const Vector2D visible_MC,
                                 const float predator_vmax,
                                 const float predator_accmax, const float ch);
  void resetacceleration();
};
};  // namespace boids_sim

#endif