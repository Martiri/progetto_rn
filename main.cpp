#include <algorithm>
#include <execution>
#include "Vector2D.hpp"
#include "boid.hpp"
#include "flock.hpp"
#include "flockconfiguration.hpp"
#include "predator.hpp"
#include "simgraphics.hpp"
#include "simvalues.hpp"
#include "slider.hpp"

int main() {
  try {
    int boids_num = 6000;
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
   // sliders.emplace_back(xPos, yStart + 6 * verticalSpacing, 150.0f, 0.0f,
 //                        sv.d * 2, sv.d, font, "d");
   // sliders.back().setValue(sv.d);
/*    sliders.emplace_back(xPos, yStart + 7 * verticalSpacing, 150.0f, 0.0f,
                         sv.escape_d * 2, sv.escape_d, font, "escape_d");
    sliders.back().setValue(sv.escape_d);
    sliders.emplace_back(xPos, yStart + 8 * verticalSpacing, 150.0f, 0.0f,
                         sv.predator_d * 2, sv.predator_d, font, "predator_d");
    sliders.back().setValue(sv.predator_d);
    sliders.emplace_back(xPos, yStart + 9 * verticalSpacing, 150.0f, 0.0f,
                         sv.vmax * 2, sv.vmax, font, "vmax");
    sliders.back().setValue(sv.vmax);
    sliders.emplace_back(xPos, yStart + 10 * verticalSpacing, 150.0f, 0.0f,
                         sv.accmax * 2, sv.accmax, font, "accmax");
    sliders.back().setValue(sv.accmax);
    sliders.emplace_back(xPos, yStart + 11 * verticalSpacing, 150.0f, 0.0f,
                         sv.predator_vmax * 2, sv.predator_vmax, font,
                         "predator vmax");
    sliders.back().setValue(sv.predator_vmax);
    sliders.emplace_back(xPos, yStart + 12 * verticalSpacing, 150.0f, 0.0f,
                         sv.predator_accmax * 2, sv.predator_accmax, font,
                         "predator accmax");
    sliders.back().setValue(sv.predator_accmax);
    sliders.emplace_back(xPos, yStart + 13 * verticalSpacing, 150.0f, 0.0f,
                         sv.maxX * 2, sv.maxX, font, "maxX");
    sliders.back().setValue(sv.maxX);
    sliders.emplace_back(xPos, yStart + 14 * verticalSpacing, 150.0f, 0.f,
                         sv.maxY * 2, sv.maxY, font, "maxY");
    sliders.back().setValue(sv.maxY);
    sliders.emplace_back(xPos, yStart + 15 * verticalSpacing, 150.0f,
                         -sv.dt * 2, sv.dt * 2, sv.dt, font, "dt"); */
   // sliders.back().setValue(sv.dt);
    // sf::Clock aclock;
    // int framec = 0;
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
       //     if (sliders[6].getValue() != sv.d)
 //       sv.flock_safe_modify_d(sliders[6].getValue(), flock);
/*      if (sliders[7].getValue() != sv.escape_d)
        sv.modify_escape_d(sliders[7].getValue());
      if (sliders[8].getValue() != sv.predator_d)
        sv.modify_predator_d(sliders[8].getValue());
      if (sliders[9].getValue() != sv.vmax)
        sv.modify_vmax(sliders[9].getValue());
      if (sliders[10].getValue() != sv.accmax)
        sv.modify_accmax(sliders[10].getValue());
      if (sliders[11].getValue() != sv.predator_vmax)
        sv.modify_predator_vmax(sliders[11].getValue());
      if (sliders[12].getValue() != sv.predator_accmax)
        sv.modify_predator_accmax(sliders[12].getValue());
      if (sliders[13].getValue() != sv.maxX)
        sv.flock_safe_modify_maxX(sliders[13].getValue(), flock);
      if (sliders[14].getValue() != sv.maxY)
        sv.flock_safe_modify_maxY(sliders[14].getValue(), flock);
      if (sliders[15].getValue() != sv.dt) sv.modify_dt(sliders[15].getValue()); */

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