#include "boid.hpp"

#include <cmath>
#include <iostream>
#include <vector>

#include "Vector2D.hpp"
#include "simvalues.hpp"

namespace boids_sim {
boid::boid(const float x, const float y, const float vx, const float vy)
    : position_{x, y}, velocity_{vx, vy}, acceleration_{0.f, 0.f} {}
const Vector2D& boid::getPosition() const { return position_; }
const Vector2D& boid::getVelocity() const { return velocity_; }
const Vector2D& boid::getAcceleration() const { return acceleration_; }
void boid::updateacceleration(const Vector2D acc) { acceleration_ = acc; }
void boid::updatevelocity(const float dt, const float vmax) {
  velocity_ += acceleration_ * dt;
  if (velocity_.norm2() > vmax * vmax) velocity_ = velocity_.scale_to(vmax);
}
void boid::updateposition(const float dt, const float maxX, const float maxY) {
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
Vector2D boid::tuneacceleration(const Vector2D desired_velocity,
                                const float accmax) const {
  Vector2D acceleration = desired_velocity - velocity_;
  if (acceleration.norm2() > accmax * accmax)
    acceleration = acceleration.scale_to(accmax);
  return acceleration;
}
void boid::collect_infos(const Vector2D& other_position,
                         const Vector2D& other_velocity, const float maxX,
                         const float maxY, const float d2, const float ds2,
                         int& neighbours_count, Vector2D& cumdist,
                         Vector2D& cumvel,
                         Vector2D& cum_weighted_shortdist) const {
  Vector2D dist = other_position.toroidal_minus(position_, maxX, maxY);
  float dist2 = dist.norm2();
  if (dist2 < d2) {
    neighbours_count++;
    cumdist += dist;
    cumvel += other_velocity;
    if (dist2 < ds2 && dist2 != 0)
      cum_weighted_shortdist += dist * (1.f / dist2);
  }
}
Vector2D boid::calculate_escaping_acceleration(
    const Vector2D predator_position, const float e, const float escape_d2,
    const float vmax, const float accmax, float maxX, const float maxY) const {
  Vector2D acce{0.f, 0.f};
  Vector2D predator_dist =
      position_.toroidal_minus(predator_position, maxX, maxY);
  float predator_dist_norm2 = predator_dist.norm2();
  if (predator_dist_norm2 < escape_d2) {
    Vector2D e_desired_velocity{0.f, 0.f};
    if (predator_dist_norm2 > 0.1f)
      e_desired_velocity =
          predator_dist.scale_to(vmax) *
          std::sqrt(escape_d2 / std::max(predator_dist_norm2, 1.f));
    acce = tuneacceleration(e_desired_velocity, accmax) * e;
  }
  return acce;
}
Vector2D boid::calculate_separation_acceleration(
    const Vector2D cum_weighted_shortdist, const float s, const float vmax,
    const float accmax) const {
  Vector2D s_desired_velocity{0.f, 0.f};
  if (cum_weighted_shortdist.norm2() > 0)
    s_desired_velocity = cum_weighted_shortdist.scale_to(vmax);
  return tuneacceleration(s_desired_velocity, accmax) * (-s);
}
Vector2D boid::calculate_alignment_acceleration(
    const Vector2D cumvel, const float inv_neighbours_count, const float a,
    const float vmax, const float accmax) const {
  Vector2D neighbours_AV = cumvel * inv_neighbours_count;
  Vector2D a_desired_velocity{0.f, 0.f};
  if (neighbours_AV.norm2() > 0)
    a_desired_velocity = neighbours_AV.scale_to(vmax);
  return tuneacceleration(a_desired_velocity, accmax) * a;
}
Vector2D boid::calculate_cohesion_acceleration(const Vector2D cumdist,
                                               const float inv_neighbours_count,
                                               const float c, const float vmax,
                                               const float accmax) const {
  Vector2D neighbours_MC_dist = cumdist * inv_neighbours_count;
  Vector2D c_desired_velocity{0.f, 0.f};
  if (neighbours_MC_dist.norm2() > 0)
    c_desired_velocity = neighbours_MC_dist.scale_to(vmax);
  return tuneacceleration(c_desired_velocity, accmax) * c;
}

};  // namespace boids_sim