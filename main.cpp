#include "SimValues.hpp"
#include "Vector2D.hpp"
#include "boid.hpp"
#include "flock.hpp"
#include "slider.hpp"

int main() {
  try {
    boids_sim::SimValues sim_values;
    sim_values.numBoids = 500;
    // float f = 4.f;
    sim_values.s = 20.f;
    sim_values.a = 2.f;
    sim_values.c = 0.5f;
    sim_values.d = 40.f;
    sim_values.ds = 5.f;
    sim_values.d2 = sim_values.d * sim_values.d;
    sim_values.ds2 = sim_values.ds * sim_values.ds;
    sim_values.factorx = 20;
    sim_values.factory = 20;
    sim_values.ncells = sim_values.factorx * sim_values.factory;
    sim_values.maxX = sim_values.d * sim_values.factorx;
    sim_values.maxY = sim_values.d * sim_values.factory;
    sim_values.dt = 0.016f;
    sim_values.timescale = 1.0f;
    sim_values.vmax = 100.f;
    sim_values.vmax2 = sim_values.vmax * sim_values.vmax;
    sim_values.accmax = 50.f;
    sim_values.accmax2 = sim_values.accmax * sim_values.accmax;
    sim_values.distV_amplitude = 200.f;
    sim_values.distV_offset = 70.f;
    sim_values.spawn_spacing_coeff = 0.5f;
    sim_values.spawn_inf_edgeX_coeff =
        0.5f - sim_values.spawn_spacing_coeff * (1.f / (2 * std::sqrt(2)));
    sim_values.spawn_sup_edgeX_coeff =
        0.5f + sim_values.spawn_spacing_coeff * (1.f / (2 * std::sqrt(2)));
    sim_values.spawn_inf_edgeY_coeff = sim_values.spawn_inf_edgeX_coeff;
    sim_values.spawn_sup_edgeY_coeff = sim_values.spawn_sup_edgeX_coeff;
    sim_values.min_cos_of_view = -0.5f;
    // assert(min_cos_of_view < -0.0001f);
    sim_values.max_cos2_of_view =
        sim_values.min_cos_of_view * sim_values.min_cos_of_view;
    sim_values.random_behaviour_intensity_coeff = 0.15f;
    sim_values.const_random_behaviour_duration = 1200;
    sim_values.var_random_behaviour_duration =
        sim_values.const_random_behaviour_duration + 1;
    sim_values.rush_v_coeff = 0.3f;
    sim_values.fatigue_v_coeff = 0.5f;
    sim_values.recovered_v_coeff = 0.8f;
    float rush_threshold_v = sim_values.vmax * sim_values.rush_v_coeff;
    float recovered_threshold_v = sim_values.vmax * sim_values.recovered_v_coeff;
    float fatigue_threshold_v = sim_values.vmax * sim_values.fatigue_v_coeff;
    sim_values.rush_threshold_v2 = rush_threshold_v * rush_threshold_v;
    sim_values.recovered_threshold_v2 = recovered_threshold_v * recovered_threshold_v;
    sim_values.fatigue_threshold_v2 = fatigue_threshold_v * fatigue_threshold_v;
    sim_values.fatigue_braking_coeff = -0.6f;
    sim_values.rush_accelerating_coeff = 0.5f;
    sim_values.patience = 50;
    sim_values.stamina = 200;
    boids_sim::flock flock(sim_values);
    // float idealv2 = idealv * idealv;
    sf::RenderWindow window(sf::VideoMode(sim_values.maxX, sim_values.maxY),
                            "Boids Simulation");
    sf::Font font;
    if (!font.loadFromFile("arial.ttf")) {
      throw std::runtime_error("Impossibile caricare il font arial.ttf");
    }

    float &s = sim_values.s;
    float &a = sim_values.a;
    float &c = sim_values.c;
    float &d = sim_values.d;
    float &ds = sim_values.ds;
    float &d2 = sim_values.d2;
    float &ds2 = sim_values.ds2;
    //  float &dt = sim_values.dt;
    //  float &timescale = sim_values.timescale;
    float &vmax = sim_values.vmax;
    float &accmax = sim_values.accmax;
    float &vmax2 = sim_values.vmax2;
    float &accmax2 = sim_values.accmax2;
    // float &numBoids = sim_values.numBoids;

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
                         accmax * 2, accmax, font, "accmax");
    sliders.back().setValue(accmax);
    sliders.emplace_back(xPos, yStart + 6 * verticalSpacing, 150.0f, 0.0f,
                         vmax * 2, vmax, font, "vmax");
    sliders.back().setValue(vmax);
    // sf::CircleShape boidShape(4.0f);
    /* int numVertices = numBoids * 3;
    sf::VertexArray boidVertices(sf::Triangles, numVertices);
    for (int i = 0; i < numVertices; i++)
      boidVertices[i].color = sf::Color::Blue; */
    sf::ConvexShape boidShape;
    boidShape.setPointCount(3);

    // La punta (naso) a destra dello zero
    boidShape.setPoint(0, sf::Vector2f(7.5f, 0.f));

    // I due punti della coda a sinistra dello zero
    boidShape.setPoint(1, sf::Vector2f(-7.5f, -5.f));
    boidShape.setPoint(2, sf::Vector2f(-7.5f, 5.f));

    // IMPORTANTE: L'origine deve essere (0,0) perché i punti sono già centrati
    boidShape.setOrigin(0.f, 0.f);

    sf::Clock clock;

    while (window.isOpen()) {
      sf::Event event;
      while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) window.close();
        for (auto &s : sliders) s.handleEvent(event, window);
      }

      //  dt = clock.restart().asSeconds();
      //  if (dt > 0.1) dt = 0.1;
      //  dt *= timescale;

      for (auto &s : sliders) s.update(window);

      s = sliders[0].getValue();
      a = sliders[1].getValue();
      c = sliders[2].getValue();
      d = sliders[3].getValue();
      ds = sliders[4].getValue();
      accmax = sliders[5].getValue();
      vmax = sliders[6].getValue();

      accmax2 = accmax * accmax;
      vmax2 = vmax * vmax;
      d2 = d * d;
      ds2 = ds * ds;
      std::vector<boids_sim::Vector2D> posizioni_vecchie;
      for (const auto &b : flock.getBoids()) {
        posizioni_vecchie.push_back(b.getPosition());
      }
      flock.step(sim_values);
      window.clear(sf::Color::Black);
      for (const auto &boid : flock.getBoids()) {
        const auto &pos = boid.getPosition();
        const auto &vel = boid.getVelocity();
        // Disegno boid
        boidShape.setPosition(pos.x, pos.y);
        boidShape.setRotation(std::atan2(vel.y, vel.x) * 57.2958f);
        window.draw(boidShape);
      }

      for (auto &s : sliders) s.draw(window);

      window.display();
      // }
    }
  } catch (const std::exception &e) {
    std::cerr << "Errore: " << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Errore sconosciuto." << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}