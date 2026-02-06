#ifndef SIMVALUES_HPP
#define SIMVALUES_HPP

#include <vector>

namespace boids_sim {

struct SimValues {
  float dt{1.f};
  float s{0.f}, a{0.f}, c{0.f}, e{0.f}, ch{0.f};
  float ds{0.f}, escape_d{0.f}, predator_d{0.f}, ds2{0.f}, escape_d2{0.f},
      predator_d2{0.f};
  float vmax{4.f}, accmax{0.f}, predator_vmax{5.f}, predator_accmax{0.f};
  const float d{55.f}, d2{3025.f}, maxX{990.f}, maxY{715.f};

  SimValues() = default;
  static SimValues StdValues();
  void modify_dt(const float _new);
  void modify_s(const float _new);
  void modify_a(const float _new);
  void modify_c(const float _new);
  void modify_e(const float _new);
  void modify_ch(const float _new);
  void modify_ds(const float _new);
  void modify_escape_d(const float _new);
  void modify_predator_d(const float _new);
  void modify_vmax(const float _new);
  void modify_accmax(const float _new);
  void modify_predator_vmax(const float _new);
  void modify_predator_accmax(const float _new);
};

};  // namespace boids_sim

#endif