#ifndef SIMVALUES_HPP
#define SIMVALUES_HPP

#include <vector>

namespace boids_sim {

class flock;

struct SimValues {
  float dt{1.f};
  float s{0.f}, a{0.f}, c{0.f}, e{0.f}, ch{0.f};
  float d{10.f}, ds{0.f}, escape_d{0.f}, predator_d{0.f}, d2{0.f},
        ds2{0.f}, escape_d2{0.f}, predator_d2{0.f};
  float vmax{0.f}, accmax{0.f}, predator_vmax{0.f}, predator_accmax{0.f};
  float maxX{100.f}, maxY{100.f};

  SimValues() = default;
  static SimValues StdValues(const float dt);
  void modify_dt(const float _new);
  void modify_s(const float _new);
  void modify_a(const float _new);
  void modify_c(const float _new);
  void modify_e(const float _new);
  void modify_ch(const float _new);
  void flock_unsafe_modify_d(const float _new);
  void flock_safe_modify_d(const float new_d, flock& flock);
  void flock_safe_modify_d(const float new_d,
                           const std::vector<flock*>& flocks);
  void modify_ds(const float _new);
  void modify_escape_d(const float _new);
  void modify_predator_d(const float _new);
  void modify_vmax(const float _new);
  void modify_accmax(const float _new);
  void modify_predator_vmax(const float _new);
  void modify_predator_accmax(const float _new);
  void flock_unsafe_modify_maxX(const float _new);
  void flock_safe_modify_maxX(const float new_maxX, flock& flock);
  void flock_safe_modify_maxX(const float new_maxX,
                              const std::vector<flock*>& flocks);
  void flock_unsafe_modify_maxY(const float _new);
  void flock_safe_modify_maxY(const float new_maxY, flock& flock);
  void flock_safe_modify_maxY(const float new_maxY,
                              const std::vector<flock*>& flocks);
};

};  // namespace boids_sim

#endif