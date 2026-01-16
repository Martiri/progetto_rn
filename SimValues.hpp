#ifndef SIMVALUES_HPP
#define SIMVALUES_HPP

namespace boids_sim {

struct SimValues {
  float numBoids;
  float dt, timescale;
  float s, a, c, e, ch;
  float d, ds, d2, ds2, b_predator_attention_coeff, escape_d, escape_d2;
  float maxX, maxY, ncells;
  int factorx, factory;
  float vmax, accmax, vmax2, accmax2;
  float spawn_spacing_coeff, spawn_inf_edgeX_coeff, spawn_sup_edgeX_coeff,
      spawn_inf_edgeY_coeff, spawn_sup_edgeY_coeff, distV_amplitude,
      distV_offset;
  float predator_vmax, predator_vmax2, predator_d, predator_d2,
      predator_bonus_accmax_coeff, predator_accmax;
  /* float min_cos_of_view;
  float max_cos2_of_view;
  float random_behaviour_intensity_coeff;
  int const_random_behaviour_duration, var_random_behaviour_duration; */

  SimValues() = default;
};

};  // namespace boids_sim

#endif