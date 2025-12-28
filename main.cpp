
#include <iomanip>
#include <iostream>
#include <sstream>
#include "boid.hpp"
#include "flock.hpp"

#include <SFML/Graphics.hpp>

int main() {
  try{
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
  float maxX = d * static_cast<float>(factorx);
  float maxY = d * static_cast<float>(factory);
  float dt = 1000.f;
  float timescale = 5.0f;
  float vmax2 = 10000.f;

  boids_sim::flock flock(numBoids, maxX, maxY, ncells);
  sf::RenderWindow window(sf::VideoMode(static_cast<unsigned int>(maxX), static_cast<unsigned int>(maxY) ), "Boids Simulation");
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
    if (dt > 0.1f) dt = 0.1f;
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
  } catch (const std::exception &e) {
    std::cerr << "Errore: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }catch (...) {
    std::cerr << "Errore sconosciuto." << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;

}
