#include <SFML/Graphics.hpp>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "flock.hpp"

int main() {
  int numBoids = 200;
  float s = 0.15f;
  float a = 0.08f;
  float c = 0.01f;
  float d = 60.0f;
  float d2 = d * d;
  float ds = 20.0f;
  float ds2 = ds * ds;
  int factorx = 20;
  int factory = 20;
  float maxX = d * factorx;
  float maxY = d * factory;
  float dt = 0.0f;
  float timescale = 1.0f;

  boids_sim::flock flock(numBoids, maxX, maxY);
  sf::RenderWindow window(sf::VideoMode(maxX, maxY), "Boids Simulation");
  window.setFramerateLimit(60);

  sf::CircleShape boidShape(4.0f);
  boidShape.setFillColor(sf::Color::Blue);
  boidShape.setOrigin(4.0f, 4.0f);

  sf::Clock clock;

  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) window.close();
    }

    double dt = clock.restart().asSeconds();
    if (dt > 0.1) dt = 0.1;
    dt *= timescale;

    flock.step(dt, factorx, s, a, c, d, ds);
    window.clear(sf::Color::Black);

    for (const auto& boid : flock.getBoids()) {
      const auto& pos = boid.getPosition();
      boidShape.setPosition(static_cast<float>(pos.x),
                            static_cast<float>(pos.y));
      window.draw(boidShape);
    }
  }
  window.display();

  return 0;
}
