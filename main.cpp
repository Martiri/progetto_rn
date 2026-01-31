#include <algorithm>
#include <execution>

#include "Vector2D.hpp"
#include "boid.hpp"
#include "flock.hpp"
#include "predator.hpp"
#include "simgraphics.hpp"
#include "simvalues.hpp"
#include "slider.hpp"

int main() {
  try {
    boids_sim::SimValues sv;
    sv.boids_num = 6000;
    sv.s = 1.9;
    sv.a = 0.8f;
    sv.c = 0.7f;
    sv.d = 55.f;
    sv.ds = 36.f;
    sv.d2 = sv.d * sv.d;
    sv.ds2 = sv.ds * sv.ds;
    sv.factorx = 18;
    sv.factory = 13;
    sv.ncells = sv.factorx * sv.factory;
    sv.maxX = sv.d * sv.factorx;
    sv.maxY = sv.d * sv.factory;
    sv.dt = 1.f;
    sv.vmax = 4.f;
    sv.vmax2 = sv.vmax * sv.vmax;
    sv.accmax = 0.18f;
    sv.accmax2 = sv.accmax * sv.accmax;
    sv.distV_amplitude = 2.8f;
    sv.distV_offset = 0.f;
    sv.spawn_spacing_coeff = 0.5f;
    sv.spawn_inf_edgeX_coeff =
        0.5f - sv.spawn_spacing_coeff * (1.f / (2 * std::sqrt(2)));
    sv.spawn_sup_edgeX_coeff =
        0.5f + sv.spawn_spacing_coeff * (1.f / (2 * std::sqrt(2)));
    sv.spawn_inf_edgeY_coeff = sv.spawn_inf_edgeX_coeff;
    sv.spawn_sup_edgeY_coeff = sv.spawn_sup_edgeX_coeff;
    sv.e = 2.5f;
    sv.ch = 1.5f;
    sv.boids_caution_coeff = 1.8f;
    sv.escape_d = sv.d * sv.boids_caution_coeff;
    sv.escape_d2 = sv.escape_d * sv.escape_d;
    sv.predator_vmax = 5.f;
    sv.predator_vmax2 = sv.predator_vmax * sv.predator_vmax;
    sv.predator_d = 180.f;
    sv.predator_d2 = sv.predator_d * sv.predator_d;
    sv.predator_bonus_accmax_coeff = 1.2f;
    sv.predator_accmax = sv.accmax * sv.predator_bonus_accmax_coeff;
    // sv.steps_counter = 0;
    boids_sim::flock flock(sv);
    boids_sim::SimGraphics sg(flock, 5.1f, 9.9f);
    sg.set_boids_color(sf::Color::White);
    sg.set_predator_color(sf::Color::Red);
    sf::RenderWindow window(sf::VideoMode(sv.maxX, sv.maxY),
                            "Boids Simulation");
    sf::Font font;
    if (!font.loadFromFile("arial.ttf")) {
      throw std::runtime_error("Impossibile caricare il font arial.ttf");
    }
    float &s = sv.s;
    float &a = sv.a;
    float &c = sv.c;
    float &ds = sv.ds;
    float &vmax = sv.vmax;
    float &accmax = sv.accmax;
    float &predator_vmax = sv.predator_vmax;
    float &predator_bonus_accmax_coeff = sv.predator_bonus_accmax_coeff;
    float &boids_caution_coeff = sv.boids_caution_coeff;
    // int &steps_counter = sv.steps_counter;

    window.setFramerateLimit(60);
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
                         ds * 2, ds, font, "ds");
    sliders.back().setValue(ds);
    sliders.emplace_back(xPos, yStart + 4 * verticalSpacing, 150.0f, 0.0f,
                         accmax * 2, accmax, font, "accmax");
    sliders.back().setValue(accmax);
    sliders.emplace_back(xPos, yStart + 5 * verticalSpacing, 150.0f, 0.0f,
                         vmax * 2, vmax, font, "vmax");
    sliders.back().setValue(vmax);
    sliders.emplace_back(xPos, yStart + 6 * verticalSpacing, 150.0f, 0.0f,
                         predator_bonus_accmax_coeff * 2,
                         predator_bonus_accmax_coeff, font,
                         "predator accmax ÷ boid accmax ");
    sliders.back().setValue(predator_bonus_accmax_coeff);
    sliders.emplace_back(xPos, yStart + 7 * verticalSpacing, 150.0f, 0.0f,
                         predator_vmax * 2, predator_vmax, font,
                         "predator vmax");
    sliders.back().setValue(predator_vmax);
    sliders.emplace_back(xPos, yStart + 8 * verticalSpacing, 150.0f, 0.0f,
                         boids_caution_coeff * 2, boids_caution_coeff, font,
                         "boids' caution");
    sliders.back().setValue(boids_caution_coeff);
    /* sf::ConvexShape boidShape;
    sf::ConvexShape predatorShape;
    predatorShape.setFillColor(sf::Color::Red);
    boidShape.setPointCount(3);
    predatorShape.setPointCount(3);
    boidShape.setPoint(0, sf::Vector2f(5.1f, 0.f));
    boidShape.setPoint(1, sf::Vector2f(-5.1f, -3.4f));
    boidShape.setPoint(2, sf::Vector2f(-5.1f, 3.4f));
    predatorShape.setPoint(0, sf::Vector2f(9.9f, 0.f));
    predatorShape.setPoint(1, sf::Vector2f(-9.9f, -6.6f));
    predatorShape.setPoint(2, sf::Vector2f(-9.9f, 6.6f));

    boidShape.setOrigin(0.f, 0.f);
    predatorShape.setOrigin(0.f, 0.f);
*/
    // sf::Clock clock;
    // sf::Clock fpsClock;
    // int frameCount = 0;

    while (window.isOpen()) {
      sf::Event event;
      while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) window.close();
        std::for_each(sliders.begin(), sliders.end(),
                      [&](auto &sl) { sl.handleEvent(event, window); });
      }
      std::for_each(sliders.begin(), sliders.end(),
                    [&window](auto &sl) { sl.update(window); });

      s = sliders[0].getValue();
      a = sliders[1].getValue();
      c = sliders[2].getValue();
      sv.modify_ds(sliders[3].getValue());
      sv.modify_accmax(sliders[4].getValue());
      sv.modify_vmax(sliders[5].getValue());
      sv.modify_predator_bonus_accmax_coeff(sliders[6].getValue());
      sv.modify_predator_vmax(sliders[7].getValue());

      flock.step(sv);
      // steps_counter++;
      window.clear(sf::Color::Black);
      sg.update();
      sg.draw(window);
      /*     for (const auto &boid : flock.getBoids()) {
             const auto &pos = boid.getPosition();
             const auto &vel = boid.getVelocity();
             // Disegno boid
             boidShape.setPosition(pos.x, pos.y);
             boidShape.setRotation(std::atan2(vel.y, vel.x) * 57.2958f);
             window.draw(boidShape);
           }
           predatorShape.setPosition(flock.getPredator().getPosition().x,
                                     flock.getPredator().getPosition().y);
           predatorShape.setRotation(
               std::atan2(flock.getPredator().getVelocity().y,
                          flock.getPredator().getVelocity().x) *
               57.2958f);
           window.draw(predatorShape);  */

      std::for_each(sliders.begin(), sliders.end(),
                    [&](auto &sl) { sl.draw(window); });

      window.display();
      // frameCount++;
      /* if (fpsClock.getElapsedTime().asSeconds() >= 1.0f) {
        float fps = frameCount / fpsClock.restart().asSeconds();
        std::cout << "Boids Simulation - FPS: " << fps << std::endl;
        frameCount = 0;
      } */
    }
  }
catch (const std::exception &exc) {
  std::cerr << "Errore: " << exc.what() << std::endl;
  return EXIT_FAILURE;
}
catch (...) {
  std::cerr << "Errore sconosciuto." << std::endl;
  return EXIT_FAILURE;
}
return EXIT_SUCCESS;
}