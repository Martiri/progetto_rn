#ifndef FLOCKCONFIGURATION_HPP
#define FLOCKCONFIGURATION_HPP

#include "vector2d.hpp"

namespace boids_sim {

struct FlockConfiguration {
  int boids_num;
  float spawn_inf_edgeX_coeff, spawn_sup_edgeX_coeff, spawn_inf_edgeY_coeff,
      spawn_sup_edgeY_coeff;
  float distV_amplitude;
  Vector2D predator_starting_position, predator_starting_velocity;

  FlockConfiguration() = default;
  static FlockConfiguration StdConfig(const int boids_number, const float maxX, const float maxY);

  void constrain(const float maxX, const float maxY, const float vmax);
};

};  // namespace boids_sim

#endif