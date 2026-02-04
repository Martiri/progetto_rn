#ifndef SIMGRAPHICS_HPP
#define SIMGRAPHICS_HPP
#include <SFML/Graphics.hpp>
#include "flock.hpp"

namespace boids_sim {

class SimGraphics {
 private:
  sf::VertexArray boids_vertices_;
  sf::ConvexShape predator_shape_;
  const flock& flock_;
  float boid_size_;

 public:
  SimGraphics() = delete;
  SimGraphics(const flock& flock, const float boid_size,
              const float predator_size);
  void update();
  void draw(sf::RenderWindow& window) const;
  void set_boids_color(const sf::Color& color);
  void set_predator_color(const sf::Color& color);
};
};  // namespace boids_sim
#endif