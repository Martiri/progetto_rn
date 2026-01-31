#include <algorithm>
#include <execution>

#include "simgraphics.hpp"

namespace boids_sim {

SimGraphics::SimGraphics(const flock& flock, const float boid_size,
                         const float predator_size)
    : flock_(flock), boid_size_(boid_size) {
  boids_vertices_.resize(flock.get_boids_num() * 3);
  boids_vertices_.setPrimitiveType(sf::Triangles);
  predator_shape_.setFillColor(sf::Color::Red);
  predator_shape_.setPointCount(3);
  predator_shape_.setPoint(0, sf::Vector2f(predator_size, 0.f));
  predator_shape_.setPoint(
      1, sf::Vector2f(-predator_size, -predator_size * 2.f / 3.f));
  predator_shape_.setPoint(
      2, sf::Vector2f(-predator_size, predator_size * 2.f / 3.f));
  predator_shape_.setOrigin(0.f, 0.f);
}
void SimGraphics::update() {
  const std::vector<boid>& boids = flock_.getBoids();
  const predator& predator = flock_.getPredator();
  std::for_each(boids.begin(), boids.end(),
                [&](const boid& boid) {
                  size_t i = &boid - &boids[0];
                  size_t vtx_i = i * 3;
                  Vector2D pos = boid.getPosition();
                  Vector2D vel = boid.getVelocity();
                  float vel_norm = std::sqrt(vel.norm2());
                  float inv_vel_norm;
                  if (vel_norm != 0)
                    inv_vel_norm = 1.f / vel_norm;
                  else
                    return;
                  float sin = vel.y * inv_vel_norm;
                  float cos = vel.x * inv_vel_norm;
                  float size = boid_size_;
                  float tt_size = size * 2.f / 3.f;
                  boids_vertices_[vtx_i].position =
                      sf::Vector2f(pos.x + cos * size, pos.y + sin * size);
                  // Retro-sinistra
                  boids_vertices_[vtx_i + 1].position =
                      sf::Vector2f(pos.x + (-size * cos + tt_size * sin),
                                   pos.y + (-5.1f * sin - tt_size * cos));
                  // Retro-destra
                  boids_vertices_[vtx_i + 2].position =
                      sf::Vector2f(pos.x + (-5.1f * cos - tt_size * sin),
                                   pos.y + (-5.1f * sin + tt_size * cos));
                });
  Vector2D pos = predator.getPosition();
  Vector2D vel = predator.getVelocity();
  predator_shape_.setPosition(pos.x, pos.y);
  predator_shape_.setRotation(std::atan2(vel.y, vel.x) * 57.2958f);
}
void SimGraphics::draw(sf::RenderWindow& window) const {
  window.draw(boids_vertices_);
  window.draw(predator_shape_);
}
void SimGraphics::set_boids_color(const sf::Color& color) {
for (int i{0}; i < flock_.get_boids_num(); i++){
    size_t vtx_i = i * 3;
    boids_vertices_[vtx_i].color = color;
    boids_vertices_[vtx_i+1].color = color;
    boids_vertices_[vtx_i+2].color = color;
}
}
void SimGraphics::set_predator_color(const sf::Color& color) {
    predator_shape_.setFillColor(color);
}
};  // namespace boids_sim