#ifndef SIMVALUES_HPP
#define SIMVALUES_HPP

namespace boids_sim {

struct SimValues {
  float boids_num;
  float dt;
  float s, a, c, e, ch;
  float d, ds, d2, ds2, boids_caution_coeff, escape_d, escape_d2;
  float maxX, maxY, ncells;
  int factorx, factory;
  float vmax, accmax, vmax2, accmax2;
  float spawn_spacing_coeff, spawn_inf_edgeX_coeff, spawn_sup_edgeX_coeff,
      spawn_inf_edgeY_coeff, spawn_sup_edgeY_coeff, distV_amplitude,
      distV_offset;
  float predator_vmax, predator_vmax2, predator_d, predator_d2,
      predator_bonus_accmax_coeff, predator_accmax;
  int steps_counter;

  SimValues() = default;
  void modify_d(float _new);
  void modify_ds(float _new);
  void modify_boids_caution_coeff(float _new);
  void modify_vmax(float _new);
  void modify_accmax(float _new);
  void modify_predator_d (float _new);
  void modify_predator_vmax(float _new);
  void modify_predator_bonus_accmax_coeff(float _new);
};

};  // namespace boids_sim

#endif