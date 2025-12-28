#include "boid.hpp"
#include "flock.hpp"
#include "boid.cpp"
#include "flock.cpp"

#include <SFML/Graphics.hpp>

int main() {
  int numBoids = 500;
  float s = 1.5f;
  float a = 0.5f;
  float c = 0.01f;
  float d = 100.0f;
  float ds = 5.0f;
  float length = d;
  float d2 = d * d;
  float ds2 = ds * ds;
  int factorx = 10;
  int factory = 10;
  int ncells = factorx * factory;
  float maxX = d * factorx;
  float maxY = d * factory;
  float dt = 1000.f;
  float timescale = 5.0f;
  float vmax2 = 10000.f;

  boids_sim::flock flock(numBoids, maxX, maxY, ncells);
  sf::RenderWindow window(sf::VideoMode(maxX, maxY), "Boids Simulation");
  // window.setFramerateLimit(60);

  sf::CircleShape boidShape(4.0f);
  boidShape.setFillColor(sf::Color::Blue);
  boidShape.setOrigin(4.0f, 4.0f);

  sf::Clock clock;

  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) window.close();
    }

    dt = clock.restart().asSeconds();
    if (dt > 0.1) dt = 0.1;
    dt *= timescale;

    flock.step(dt, maxX, maxY, factorx, s, a, c, d2, ds2, length, ncells, vmax2);
    window.clear(sf::Color::Black);

    for (const auto &boid : flock.getBoids()) {
      const auto &pos = boid.getPosition();
      boidShape.setPosition(static_cast<float>(pos.x),
                            static_cast<float>(pos.y));
      window.draw(boidShape);
    }
    window.display();
  }

  return 0;
}