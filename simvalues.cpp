#include "simvalues.hpp"

namespace boids_sim {

void SimValues::modify_ds(float _new) {
    ds = _new;
    ds2 = ds*ds;
}
void SimValues::modify_boids_caution_coeff(float _new) {
    boids_caution_coeff = _new;
    escape_d = d * boids_caution_coeff;
    escape_d2 = escape_d * escape_d;
}
void SimValues::modify_vmax(float _new) {
    vmax = _new;
    vmax2 = vmax * vmax;
}
void SimValues::modify_accmax(float _new) {
    accmax = _new;
    accmax2 = accmax * accmax;
}
void SimValues::modify_predator_d(float _new) {
    predator_d = _new;
    predator_d2 = predator_d * predator_d;
}
void SimValues::modify_predator_vmax(float _new) {
    predator_vmax = _new;
    predator_vmax2 = predator_vmax * predator_vmax;
}
void SimValues::modify_predator_bonus_accmax_coeff(float _new) {
    predator_bonus_accmax_coeff = _new;
    predator_accmax = accmax * predator_bonus_accmax_coeff;
}

};  // namespace boids_sim