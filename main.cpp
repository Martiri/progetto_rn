

#include "boid.hpp"
#include "flock.hpp"
#include "slider.hpp"

int main() {
  try {
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
    float vmax2 = 1000.f;

    boids_sim::flock flock(numBoids, maxX, maxY, ncells);
    sf::RenderWindow window(sf::VideoMode(static_cast<unsigned int>(maxX),
                                          static_cast<unsigned int>(maxY)),
                            "Boids Simulation");
    sf::Font font;
    if (!font.loadFromFile("arial.ttf")) {
      throw std::runtime_error("Impossibile caricare il font arial.ttf");
    }
    // window.setFramerateLimit(60);
    std::vector<boids_sim::slider> sliders;
    float xPos = 50.0f;
    float yStart = 45.0f;
    float verticalSpacing = 35.0f;
    sliders.emplace_back(xPos, yStart + 0 * verticalSpacing, 150.0f, 0.0f, 3.0f, s,
                         font, "s");
    sliders.back().setValue(s);
    sliders.emplace_back(xPos, yStart + 1 * verticalSpacing, 150.0f, 0.0f, 2.0f,
                         a, font, "a");
    sliders.back().setValue(a);
    sliders.emplace_back(xPos, yStart + 2 * verticalSpacing, 150.0f, 0.0f,
                         1.0f, c, font, "c");
    sliders.back().setValue(c);
    sliders.emplace_back(xPos, yStart + 3 * verticalSpacing, 150.0f, 0.0f,
                         150.0f, d, font, "d");
    sliders.back().setValue(d);
    sliders.emplace_back(xPos, yStart + 4 * verticalSpacing, 150.0f, 0.0f,
                         30.0f, ds, font, "ds");
    sliders.back().setValue(ds);
    sliders.emplace_back(xPos, yStart + 5 * verticalSpacing, 150.0f, 0.0f,
                         20000.0f, vmax2, font, "vmax2");
    sliders.back().setValue(vmax2);

    sf::CircleShape boidShape(4.0f);
    boidShape.setFillColor(sf::Color::Blue);
    boidShape.setOrigin(4.0f, 4.0f);

    sf::Clock clock;

    while (window.isOpen()) {
      sf::Event event;
      while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) window.close();
        for (auto& s : sliders) s.handleEvent(event, window);
      }

      dt = clock.restart().asSeconds();
      if (dt > 0.1f) dt = 0.1f;
      dt *= timescale;

      for (auto& s : sliders) s.update(window);

      s = sliders[0].getValue();
      a = sliders[1].getValue();
      c = sliders[2].getValue();
      d = sliders[3].getValue();
      ds = sliders[4].getValue();
      vmax2 = sliders[5].getValue();

      d2 = d * d;
      ds2 = ds * ds;

      flock.step(dt, maxX, maxY, factorx, s, a, c, d2, ds2, length, ncells,
                 vmax2);

      window.clear(sf::Color::Black);

      for (const auto& boid : flock.getBoids()) {
        const auto& pos = boid.getPosition();
        boidShape.setPosition(static_cast<float>(pos.x),
                              static_cast<float>(pos.y));
        window.draw(boidShape);
      }

      for (auto& s : sliders) s.draw(window);

      window.display();
    }
  } catch (const std::exception& e) {
    std::cerr << "Errore: " << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Errore sconosciuto." << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
