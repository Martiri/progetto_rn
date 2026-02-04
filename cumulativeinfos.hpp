#ifndef COLLECTIVEINFOS_HPP
#define COLLECTIVEINFOS_HPP

#include "Vector2D.hpp"

namespace boids_sim {
struct CumulativeInfos {
  int neighbors_count{0};
  Vector2D cumdist{0.f, 0.f};
  Vector2D cumvel{0.f, 0.f};
  Vector2D cum_weighted_shortdist{0.f, 0.f};

  CumulativeInfos() = default;
};
};  // namespace boids_sim

#endif