#include "flockconfiguration.hpp"

#include <algorithm>
#include <cmath>

namespace boids_sim {

FlockConfiguration FlockConfiguration::StdConfig(const int boids_number, const float maxX, const float maxY) {
  FlockConfiguration fc;
  fc.boids_num = boids_number;
  fc.distV_amplitude = 2.8f;
  fc.distV_offset = 0.f;
  float spawn_spacing_coeff = 0.5f;
  fc.spawn_inf_edgeX_coeff =
      0.5f - spawn_spacing_coeff * (1.f / (2.f * static_cast<float>(std::sqrt(2.))));
  fc.spawn_sup_edgeX_coeff =
      0.5f + spawn_spacing_coeff * (1.f / (2.f * static_cast<float>(std::sqrt(2.))));
  fc.spawn_inf_edgeY_coeff = fc.spawn_inf_edgeX_coeff;
  fc.spawn_sup_edgeY_coeff = fc.spawn_sup_edgeX_coeff;
  fc.predator_starting_position = {maxX / 2.f, maxY / 2.f};
  fc.predator_starting_velocity = {0.f, 0.f};
  return fc;
}

void FlockConfiguration::constrain() {
  distV_amplitude = std::max(0.0f, distV_amplitude);
  auto fix_range = [](float& inf, float& sup) {
    inf = std::clamp(inf, 0.01f, 0.98f);
    sup = std::clamp(sup, 0.02f, 0.99f);
    if (inf > sup)
      std::swap(inf, sup);
    else if (inf == sup)
      inf -= 0.001f;
  };
  fix_range(spawn_inf_edgeX_coeff, spawn_sup_edgeX_coeff);
  fix_range(spawn_inf_edgeY_coeff, spawn_sup_edgeY_coeff);
  boids_num = std::max(0, boids_num);
}
};  // namespace boids_sim