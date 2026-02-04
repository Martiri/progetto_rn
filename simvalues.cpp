#include <algorithm>
#include <cmath>
#include <vector>
#include "simvalues.hpp"
#include "flock.hpp"

namespace boids_sim {

SimValues SimValues::StdValues(const float dt) {
  SimValues sv{};
  sv.dt = dt;
  sv.s = 1.9;
  sv.a = 0.8f;
  sv.c = 0.7f;
  sv.e = 2.5f;
  sv.ch = 1.5f;
  sv.d = 55.f;
  sv.ds = 36.f;
  sv.escape_d = sv.d * 1.8f;
  sv.predator_d = 180.f;
  sv.d2 = sv.d * sv.d;
  sv.ds2 = sv.ds * sv.ds;
  sv.escape_d2 = sv.escape_d * sv.escape_d;
  sv.predator_d2 = sv.predator_d * sv.predator_d;
  sv.vmax = 4.f;
  sv.accmax = 0.18f;
  sv.predator_vmax = 5.f;
  sv.predator_accmax = sv.accmax * 1.2f;
  sv.maxX = 990.f;
  sv.maxY = 715.f;
  return sv;
}
void SimValues::modify_dt(const float _new) {
  float max_v = std::max(std::abs(vmax), std::abs(predator_vmax));
  if (max_v != 0) {
    float max_abs_dt{std::min(maxX / max_v, maxY / max_v)};
    dt = std::clamp(_new, -max_abs_dt, max_abs_dt);
  } else
    dt = _new;
}
void SimValues::modify_s(const float _new) { s = _new; }
void SimValues::modify_a(const float _new) { a = _new; }
void SimValues::modify_c(const float _new) { c = _new; }
void SimValues::modify_e(const float _new) { e = _new; }
void SimValues::modify_ch(const float _new) { ch = _new; }
void SimValues::flock_unsafe_modify_d(const float _new) {
  d = std::max({_new, std::sqrt(maxX * maxY / 1e4f), 1e-5f}); // massimo di 1e4 celle, d!=0
  d2 = d * d;
}
void SimValues::flock_safe_modify_d(const float new_d, flock& flock) {
  flock_unsafe_modify_d(new_d);
  flock.update_grid(d, maxX, maxY);
}
void SimValues::flock_safe_modify_d(const float new_d,
                                    const std::vector<flock*>& flocks) {
  flock_unsafe_modify_d(new_d);
  std::for_each(flocks.begin(), flocks.end(), [this](flock* flock) {
    if (flock) flock->update_grid(d, maxX, maxY);
  });
}
void SimValues::modify_ds(const float _new) {
  ds = _new;
  ds2 = ds * ds;
}
void SimValues::modify_escape_d(const float _new) {
  escape_d = _new;
  escape_d2 = escape_d * escape_d;
}
void SimValues::modify_predator_d(const float _new) {
  predator_d = _new;
  predator_d2 = predator_d * predator_d;
}
void SimValues::flock_unsafe_modify_maxX(const float _new) {
  maxX = std::max(1e-5f, std::clamp(_new,
                                    std::max(std::abs(vmax * dt),
                                             std::abs(predator_vmax * dt)),
                                    d * d * 1e4f / maxY));
}
void SimValues::flock_safe_modify_maxX(const float new_maxX, flock& flock) {
  float old_maxX{maxX};
  flock_unsafe_modify_maxX(new_maxX);
  flock.update_grid(d, maxX, maxY);
  flock.resettleX(old_maxX, maxX);
}
void SimValues::flock_safe_modify_maxX(const float new_maxX,
                                       const std::vector<flock*>& flocks) {
  float old_maxX{maxX};
  flock_unsafe_modify_maxX(new_maxX);
  std::for_each(flocks.begin(), flocks.end(), [this, old_maxX](flock* flock) {
    if (flock) {
      flock->update_grid(d, maxX, maxY);
      flock->resettleX(old_maxX, maxX);
    }
  });
}
void SimValues::flock_unsafe_modify_maxY(const float _new) {
  maxY = std::max(1e-5f, std::clamp(_new,
                                    std::max(std::abs(vmax * dt),
                                             std::abs(predator_vmax * dt)),
                                    d * d * 1e4f / maxX));
}
void SimValues::flock_safe_modify_maxY(const float new_maxY, flock& flock) {
  float old_maxY{maxY};
  flock_unsafe_modify_maxY(new_maxY);
  flock.update_grid(d, maxX, maxY);
  flock.resettleY(old_maxY, maxY);
}
void SimValues::flock_safe_modify_maxY(const float new_maxY,
                                       const std::vector<flock*>& flocks) {
  float old_maxY = maxY;
  flock_unsafe_modify_maxY(new_maxY);
  std::for_each(flocks.begin(), flocks.end(), [old_maxY, this](flock* flock) {
    if (flock) {
      flock->update_grid(d, maxX, maxY);
      flock->resettleY(old_maxY, maxY);
    }
  });
}
void SimValues::modify_vmax(const float _new) {
  if (dt != 0.f)
    vmax = std::clamp(_new, 0.f, std::min(maxX / dt, maxY / dt));
  else
    vmax = std::max(0.f, _new);
}
void SimValues::modify_accmax(const float _new) { accmax = _new; }
void SimValues::modify_predator_vmax(const float _new) {
  if (dt != 0.f)
    predator_vmax = std::clamp(_new, 0.f, std::min(maxX / dt, maxY / dt));
  else  
    predator_vmax = std::max(0.f, _new);
}
void SimValues::modify_predator_accmax(const float _new) {
  predator_accmax = _new;
}
};  // namespace boids_sim