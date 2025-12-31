

#include "boid.hpp"
#include "flock.hpp"
#include "slider.hpp"

int main() {
  try {
    try {
      int numBoids = 400;
      // float f = 4.f;
      float s = 5.0f;
      float a = 1.5f;
      float c = 1.f;
      float d = 100.0f;
      float ds = 5.0f;
      float length = d;
      float d2 = d * d;
      float ds2 = ds * ds;
      int factorx = 9;
      int factory = 9;
      int ncells = factorx * factory;
      float maxX = length * factorx;
      float maxY = length * factory;
      float dt = 0.016f;
      float timescale = 1.0f;
      float vmax = 250.f;
      float idealv = vmax * 0.5;
      float staminaf = 0.5f;
      float lazinessf = 0.3f;
      float comebackf = 0.8f;
      float fatigue_threshold_v = vmax * staminaf;
      float rush_threshold_v = vmax * lazinessf;
      float comeback_threshold_v = vmax * comebackf;
      int stamina = 200;
      int patience = 50;
      float vmax2 = vmax * vmax;
      float accmax = 120.f;
      float accmax2 = accmax * accmax;

      boids_sim::flock flock(numBoids, maxX, maxY, ncells, idealv,
                             fatigue_threshold_v, rush_threshold_v,
                             comeback_threshold_v);
      // float idealv2 = idealv * idealv;
      sf::RenderWindow window(sf::VideoMode(maxX, maxY), "Boids Simulation");
      sf::Font font;
      if (!font.loadFromFile("arial.ttf")) {
        throw std::runtime_error("Impossibile caricare il font arial.ttf");
      }
      // window.setFramerateLimit(60);
      std::vector<boids_sim::slider> sliders;
      float xPos = 50.0f;
      float yStart = 45.0f;
      float verticalSpacing = 35.0f;
      sliders.emplace_back(xPos, yStart + 0 * verticalSpacing, 150.0f, 0.0f,
                           s * 2, s, font, "s");
      sliders.back().setValue(s);
      sliders.emplace_back(xPos, yStart + 1 * verticalSpacing, 150.0f, 0.0f,
                           a * 2, a, font, "a");
      sliders.back().setValue(a);
      sliders.emplace_back(xPos, yStart + 2 * verticalSpacing, 150.0f, 0.0f,
                           c * 2, c, font, "c");
      sliders.back().setValue(c);
      sliders.emplace_back(xPos, yStart + 3 * verticalSpacing, 150.0f, 0.0f,
                           d * 2, d, font, "d");
      sliders.back().setValue(d);
      sliders.emplace_back(xPos, yStart + 4 * verticalSpacing, 150.0f, 0.0f,
                           ds * 2, ds, font, "ds");
      sliders.back().setValue(ds);
      sliders.emplace_back(xPos, yStart + 5 * verticalSpacing, 150.0f, 0.0f,
                           vmax * 2, vmax, font, "vmax");
      sliders.back().setValue(vmax);
      sf::CircleShape boidShape(4.0f);
      boidShape.setFillColor(sf::Color::Blue);
      boidShape.setOrigin(4.0f, 4.0f);

      sf::Clock clock;

      while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
          if (event.type == sf::Event::Closed) window.close();
          for (auto &s : sliders) s.handleEvent(event, window);
        }

        dt = clock.restart().asSeconds();
        if (dt > 0.1) dt = 0.1;
        dt *= timescale;

        for (auto &s : sliders) s.update(window);

        s = sliders[0].getValue();
        a = sliders[1].getValue();
        c = sliders[2].getValue();
        d = sliders[3].getValue();
        ds = sliders[4].getValue();
        vmax = sliders[5].getValue();

        vmax2 = vmax * vmax;
        d2 = d * d;
        ds2 = ds * ds;
        flock.step(dt, maxX, maxY, factorx, factory, s, a, c, d2, ds2, length,
                   vmax, vmax2, accmax, accmax2);
        window.clear(sf::Color::Black);

        for (const auto &boid : flock.getBoids()) {
          const auto &pos = boid.getPosition();
          boidShape.setPosition(static_cast<float>(pos.x),
                                static_cast<float>(pos.y));
          window.draw(boidShape);
          // std::cout << "vel :" << boid.getVelocity().x << ", " <<
          // boid.getVelocity().y << std::endl;
        }

        for (auto &s : sliders) s.draw(window);

        window.display();
      }
    } catch (const std::exception &e) {
      std::cerr << "Errore: " << e.what() << std::endl;
      return EXIT_FAILURE;
    } catch (...) {
      std::cerr << "Errore sconosciuto." << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  } catch (const std::exception &e) {
    std::cerr << "Errore: " << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Errore sconosciuto." << std::endl;
    return EXIT_FAILURE;
  }
}
