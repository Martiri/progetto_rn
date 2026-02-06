#include "flockconfiguration.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace boids_sim {

FlockConfiguration FlockConfiguration::StdConfig(const int boids_number,
                                                 const float maxX,
                                                 const float maxY) {
  FlockConfiguration fc;
  fc.boids_num = boids_number;
  fc.distV_amplitude = 2.8f;
  float spawn_spacing_coeff = 0.5f;
  fc.spawn_inf_edgeX_coeff =
      0.5f - spawn_spacing_coeff * (1.f / (2.f * std::sqrt(2.f)));
  fc.spawn_sup_edgeX_coeff =
      0.5f + spawn_spacing_coeff * (1.f / (2.f * std::sqrt(2.f)));
  fc.spawn_inf_edgeY_coeff = fc.spawn_inf_edgeX_coeff;
  fc.spawn_sup_edgeY_coeff = fc.spawn_sup_edgeX_coeff;
  fc.predator_starting_position = {maxX / 2, maxY / 2};
  fc.predator_starting_velocity = {0.f, 0.f};
  return fc;
}

void FlockConfiguration::constrain(const float maxX, const float maxY,
                                   const float vmax) {
  distV_amplitude = std::clamp(distV_amplitude, 0.f, vmax/static_cast<float>(std::sqrt(2)));
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
  predator_starting_position.x =
      std::clamp(predator_starting_position.x, 0.001f, maxX - 0.001f);
  predator_starting_position.y =
      std::clamp(predator_starting_position.y, 0.001f, maxY - 0.001f);
}

int get_valid_boids_number(std::istream& is, std::ostream& os) {
  int boids_num;
  std::string line;
  while (true) {
    if (!std::getline(is, line)) throw std::runtime_error("Input interrotto");
    std::stringstream ss(line);
    if (ss >> boids_num) {
      char c;
      if (!(ss >> c) && boids_num > 1) {
        return boids_num;
      }
    }
    os << "Per favore inserisca un numero intero valido" << std::endl;
    os << "Riprovi:" << std::endl;
  }
}
};  // namespace boids_sim