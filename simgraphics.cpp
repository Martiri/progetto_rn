#include "simgraphics.hpp"
#include <cmath>
#include <algorithm>
#include <execution>

namespace boids_sim {

SimGraphics::SimGraphics(const flock& flock, const float boid_size,
                         const float predator_size)
    : flock_(flock), boid_size_(boid_size) {
  boids_vertices_.resize(flock.getBoids().size() * 3);
  boids_vertices_.setPrimitiveType(sf::Triangles);
  predator_shape_.setFillColor(sf::Color::Red);
  predator_shape_.setPointCount(3);
  const float tt_predator_size{predator_size * 2.f / 3.f};
  predator_shape_.setPoint(0, sf::Vector2f(predator_size, 0.f));
  predator_shape_.setPoint(1, sf::Vector2f(-predator_size, -tt_predator_size));
  predator_shape_.setPoint(2, sf::Vector2f(-predator_size, tt_predator_size));
  predator_shape_.setOrigin(0.f, 0.f);
}
void SimGraphics::update() {
  std::for_each(flock_.getBoids().begin(), flock_.getBoids().end(),
                [this](const boid& boid) {
                  const size_t i{flock_.get_boid_index(boid)};
                  const size_t vtx_i{i * 3};
                  const Vector2D pos{boid.getPosition()};
                  const Vector2D vel{boid.getVelocity()};
                  const float vel_norm = std::sqrt(vel.norm2());
                  if (vel_norm <= 0.f) return;
                  float sin{vel.y / vel_norm};
                  float cos{vel.x / vel_norm};
                  float size{boid_size_};
                  float tt_size{size * 2.f / 3.f};
                  boids_vertices_[vtx_i].position =
                      sf::Vector2f(pos.x + cos * size, pos.y + sin * size);
                  boids_vertices_[vtx_i + 1].position =
                      sf::Vector2f(pos.x + (-size * cos + tt_size * sin),
                                   pos.y + (-5.1f * sin - tt_size * cos));
                  boids_vertices_[vtx_i + 2].position =
                      sf::Vector2f(pos.x + (-5.1f * cos - tt_size * sin),
                                   pos.y + (-5.1f * sin + tt_size * cos));
                });
  predator_shape_.setPosition(flock_.getPredator().getPosition().x,
                              flock_.getPredator().getPosition().y);
  predator_shape_.setRotation(std::atan2(flock_.getPredator().getVelocity().y,
                                         flock_.getPredator().getVelocity().x) *
                              57.2958f);
}
void SimGraphics::draw(sf::RenderWindow& window) const {
  window.draw(boids_vertices_);
  window.draw(predator_shape_);
}
void SimGraphics::set_boids_color(const sf::Color& color) {
  for (size_t i{0}; i < flock_.getBoids().size(); i++) {
    size_t vtx_i = i * 3;
    boids_vertices_[vtx_i].color = color;
    boids_vertices_[vtx_i + 1].color = color;
    boids_vertices_[vtx_i + 2].color = color;
  }
}
void SimGraphics::set_predator_color(const sf::Color& color) {
  predator_shape_.setFillColor(color);
}
};  // namespace boids_sim