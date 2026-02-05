#include <algorithm>
#include <limits>
#include "flock.hpp"
#include "flockconfiguration.hpp"
#include "simgraphics.hpp"
#include "simvalues.hpp"
#include "slider.hpp"
#include <iostream>
#include <vector>


int main() {
  try {
    std::cout << "Numero iniziale dei boids:" << std::endl;
    int boids_num;
    while(!(std::cin >> boids_num) || boids_num <= 0) {
      std::cout << "Per favore inserisca un numero intero valido" << std::endl;
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      std::cout << "Riprovi:" << std::endl;
    }
    float dt = 1.f;
    boids_sim::SimValues sv = boids_sim::SimValues::StdValues(dt);
    boids_sim::FlockConfiguration fc =
        boids_sim::FlockConfiguration::StdConfig(boids_num, sv.maxX, sv.maxY);
    boids_sim::flock flock(fc, sv);
    boids_sim::SimGraphics sg(flock, 5.1f, 9.9f);
    sg.set_boids_color(sf::Color::White);
    sg.set_predator_color(sf::Color::Red);
    sf::RenderWindow window(sf::VideoMode(static_cast<unsigned int>(sv.maxX),
                                          static_cast<unsigned int>(sv.maxY)),
                            "Boids Simulation");
    sf::Font font;
    if (!font.loadFromFile("arial.ttf")) {
      throw std::runtime_error("Impossibile caricare il font arial.ttf");
    }
    window.setFramerateLimit(60);
    std::vector<boids_sim::slider> sliders;
    float xPos = 50.0f;
    float yStart = 45.0f;
    float verticalSpacing = 35.0f;
    sliders.emplace_back(xPos, yStart + 0 * verticalSpacing, 150.0f, 0.0f,
                         sv.s * 2, sv.s, font, "s");
    sliders.back().setValue(sv.s);
    sliders.emplace_back(xPos, yStart + 1 * verticalSpacing, 150.0f, 0.0f,
                         sv.a * 2, sv.a, font, "a");
    sliders.back().setValue(sv.a);
    sliders.emplace_back(xPos, yStart + 2 * verticalSpacing, 150.0f, 0.0f,
                         sv.c * 2, sv.c, font, "c");
    sliders.back().setValue(sv.c);
    sliders.emplace_back(xPos, yStart + 3 * verticalSpacing, 150.0f, 0.0f,
                         sv.e * 2, sv.e, font, "e");
    sliders.back().setValue(sv.e);
    sliders.emplace_back(xPos, yStart + 4 * verticalSpacing, 150.0f, 0.0f,
                         sv.ch * 2, sv.ch, font, "ch");
    sliders.back().setValue(sv.ch);
        sliders.emplace_back(xPos, yStart + 5 * verticalSpacing, 150.0f, 0.0f,
                         sv.ds * 2, sv.ds, font, "ds");
    sliders.back().setValue(sv.ds);
    while (window.isOpen()) {
      sf::Event event;
      while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) window.close();
        std::for_each(sliders.begin(), sliders.end(),
                      [&event, &window](auto &slider) {
                        slider.handleEvent(event, window);
                      });
      }
      std::for_each(sliders.begin(), sliders.end(),
                    [&window](auto &slider) { slider.update(window); });

      if (sliders[0].getValue() != sv.s) sv.modify_s(sliders[0].getValue());
      if (sliders[1].getValue() != sv.a) sv.modify_a(sliders[1].getValue());
      if (sliders[2].getValue() != sv.c) sv.modify_c(sliders[2].getValue());
      if (sliders[3].getValue() != sv.e) sv.modify_e(sliders[3].getValue());
      if (sliders[4].getValue() != sv.ch) sv.modify_ch(sliders[4].getValue());
      if (sliders[5].getValue() != sv.ds) sv.modify_ds(sliders[5].getValue());
      flock.step(sv);
      window.clear(sf::Color::Black);
      sg.update();
      sg.draw(window);
      std::for_each(sliders.begin(), sliders.end(),
                    [&window](auto &slider) { slider.draw(window); });
      window.display();
    }
  } catch (const std::exception &exc) {
    std::cerr << "Errore: " << exc.what() << std::endl;
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Errore sconosciuto." << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}