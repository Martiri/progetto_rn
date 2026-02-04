#include "boid.hpp"
#include <cmath>


namespace boids_sim {

boid::boid(const float x, const float y, const float vx, const float vy)
    : position_{x, y}, velocity_{vx, vy} {}

Vector2D boid::getPosition() const { return position_; }
Vector2D boid::getVelocity() const { return velocity_; }

void boid::collect_infos(const Vector2D other_position,
                         const Vector2D other_velocity, const SimValues& sv,
                         CumulativeInfos& cum_infos) const {
  Vector2D dist{other_position.toroidal_minus(position_, sv.maxX, sv.maxY)};
  float dist2{dist.norm2()};
  if (dist2 < sv.d2) {
    cum_infos.neighbors_count++;
    cum_infos.cumdist += dist;
    cum_infos.cumvel += other_velocity;
    if (dist2 < sv.ds2 && dist2 > 0.f)
      cum_infos.cum_weighted_shortdist += dist / dist2;
  }
}
Vector2D boid::tune_acceleration(const Vector2D desired_velocity,
                                 const float accmax) const {
  Vector2D acceleration{desired_velocity - velocity_};
  if (acceleration.norm2() > accmax * accmax)
    acceleration = acceleration.scale_to(accmax);
  return acceleration;
}
Vector2D boid::calculate_escaping_acceleration(const Vector2D predator_position,
                                               const SimValues& sv) const {
  Vector2D e_acc{0.f, 0.f};
  Vector2D predator_dist{
      position_.toroidal_minus(predator_position, sv.maxX, sv.maxY)};
  float predator_dist_norm2{predator_dist.norm2()};
  if (predator_dist_norm2 < sv.escape_d2) {
    Vector2D e_desired_velocity{
        predator_dist.scale_to(sv.vmax) *
        std::sqrt(sv.escape_d2 / std::max(predator_dist_norm2, 1.f))};
    e_acc = tune_acceleration(e_desired_velocity, sv.accmax) * sv.e;
  }
  return e_acc;
}
Vector2D boid::calculate_separation_acceleration(
    const Vector2D cum_weighted_shortdist, const float s, const float vmax,
    const float accmax) const {
  Vector2D s_desired_velocity{cum_weighted_shortdist.scale_to(vmax)};
  return tune_acceleration(s_desired_velocity, accmax) * (-s);
}
Vector2D boid::calculate_alignment_acceleration(
    const Vector2D neighbors_average_velocity, const float a, const float vmax,
    const float accmax) const {
  Vector2D a_desired_velocity{neighbors_average_velocity.scale_to(vmax)};
  return tune_acceleration(a_desired_velocity, accmax) * a;
}
Vector2D boid::calculate_cohesion_acceleration(
    const Vector2D vector_to_center_of_mass, const float c, const float vmax,
    const float accmax) const {
  Vector2D c_desired_velocity{vector_to_center_of_mass.scale_to(vmax)};
  return tune_acceleration(c_desired_velocity, accmax) * c;
}
void boid::update_velocity(const Vector2D acceleration, const float dt,
                           const float vmax) {
  velocity_ += acceleration * dt;
  if (velocity_.norm2() > vmax * vmax) velocity_ = velocity_.scale_to(vmax);
}
void boid::update_position(const float dt, const float maxX, const float maxY) {
  position_ += velocity_ * dt;
  if (position_.x < 0) {
    position_.x += maxX;
  } else if (position_.x >= maxX) {
    position_.x -= maxX;
  }
  if (position_.y < 0) {
    position_.y += maxY;
  } else if (position_.y >= maxY) {
    position_.y -= maxY;
  }
}
void boid::resettleX(const float old_maxX, const float new_maxX) {
  (position_.x /= old_maxX) *= new_maxX;
}
void boid::resettleY(const float old_maxY, const float new_maxY) {
  (position_.y /= old_maxY) *= new_maxY;
}
};  // namespace boids_sim