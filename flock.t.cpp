#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "flock.hpp"

#include <SFML/Graphics.hpp>
#include <vector>

#include "boid.hpp"
#include "doctest.h"
#include "flockconfiguration.hpp"
#include "predator.hpp"
#include "simvalues.hpp"
#include "slider.hpp"
#include "vector2d.hpp"

TEST_CASE("Test di Simulazione Griglia e Stormo") {
  boids_sim::SimValues sim_values;
  boids_sim::FlockConfiguration flock_config;
  flock_config.boids_num = 500;
  flock_config.spawn_inf_edgeX_coeff = 0.f;
  flock_config.spawn_sup_edgeX_coeff = 1.f;
  flock_config.spawn_inf_edgeY_coeff = 0.f;
  flock_config.spawn_sup_edgeY_coeff = 1.f;
  flock_config.distV_amplitude = 2.f;
  flock_config.distV_offset = 0.f;
  flock_config.predator_starting_position = boids_sim::Vector2D{500.f, 500.f};
  flock_config.predator_starting_velocity = boids_sim::Vector2D{0.f, 0.f};

  boids_sim::flock test_flock(flock_config, sim_values);
  SUBCASE("Calcolo Cella della Griglia") {
    boids_sim::Vector2D position_top_left{50.f, 50.f};
    boids_sim::Vector2D position_bottom_right{950.f, 700.f};
    boids_sim::Vector2D position_random{7.5f, 92.3f};
    int cell_index_1 = test_flock.getcell(position_top_left, 55.f);
    int cell_index_2 = test_flock.getcell(position_bottom_right, 55.f);
    CHECK(cell_index_1 == 0);
    CHECK(cell_index_2 == 245);
    CHECK(test_flock.getcell(position_random, 100.f) == 0);
  }
  SUBCASE("Eccezione Dimensione Cella Negativa") {
    boids_sim::Vector2D position{50.f, 50.f};
    CHECK_THROWS(test_flock.getcell(position, -100.f));
  }
  SUBCASE("Eccezione Dimensione Cella Zero") {
    boids_sim::Vector2D position{50.f, 50.f};
    CHECK_THROWS(test_flock.getcell(position, 0.f));
  }
  SUBCASE("Calcolo Distanza Toroidale") {
    boids_sim::boid test_boid(100.f, 100.f, 0.f, 0.f);
    boids_sim::Vector2D position_a{150.f, 150.f};
    boids_sim::Vector2D distance_a = position_a.toroidal_minus(
        test_boid.getPosition(), sim_values.maxX, sim_values.maxY);
    CHECK(distance_a.x == doctest::Approx(50.f));
    CHECK(distance_a.y == doctest::Approx(50.f));
    boids_sim::Vector2D position_b{950.f, 100.f};
    boids_sim::Vector2D distance_b = position_b.toroidal_minus(
        test_boid.getPosition(), sim_values.maxX, sim_values.maxY);
    CHECK(distance_b.x == doctest::Approx(-140.f));
    CHECK(distance_b.y == doctest::Approx(0.f));
    boids_sim::Vector2D position_c{100.f, 950.f};
    boids_sim::Vector2D distance_c = position_c.toroidal_minus(
        test_boid.getPosition(), sim_values.maxX, sim_values.maxY);
    CHECK(distance_c.x == doctest::Approx(0.f));
    CHECK(distance_c.y == doctest::Approx(135.f));
  }
  SUBCASE("Esecuzione Step Simulazione") {
    sim_values.ds = 50.f;
    sim_values.ds2 = sim_values.ds * sim_values.ds;
    sim_values.vmax = 5.f;
    sim_values.accmax = 1.f;
    sim_values.s = 1.f;
    sim_values.a = 1.f;
    sim_values.c = 1.f;
    sim_values.e = 1.f;
    sim_values.ch = 1.f;
    sim_values.escape_d = 200.f;
    sim_values.escape_d2 = sim_values.escape_d * sim_values.escape_d;
    sim_values.predator_vmax = 5.f;
    sim_values.predator_d = 150.f;
    sim_values.predator_d2 = sim_values.predator_d * sim_values.predator_d;
    sim_values.dt = 0.016f;
    std::vector<boids_sim::Vector2D> initial_velocities;
    std::vector<boids_sim::Vector2D> initial_positions;
    for (const auto& b : test_flock.getBoids()) {
      initial_velocities.push_back(b.getVelocity());
      initial_positions.push_back(b.getPosition());
    }
    test_flock.step(sim_values);
    const auto& boids_after = test_flock.getBoids();
    int velocities_changed{0};
    int positions_changed{0};
    for (size_t i = 0; i < initial_velocities.size(); ++i) {
      boids_sim::Vector2D vel_diff =
          boids_after[i].getVelocity() - initial_velocities[i];
      if (vel_diff.norm2() > 0.0001f) {
        velocities_changed++;
      }
      boids_sim::Vector2D pos_diff =
          boids_after[i].getPosition().toroidal_minus(
              initial_positions[i], sim_values.maxX, sim_values.maxY);
      if (pos_diff.norm2() > 0.0001f) {
        positions_changed++;
      }
    }
    CHECK(velocities_changed > static_cast<int>(boids_after.size() / 2));
    CHECK(positions_changed > static_cast<int>(boids_after.size() / 2));
  }
}
TEST_CASE("Test Operatori Vector2D") {
  boids_sim::Vector2D v1{3.f, 4.f};
  boids_sim::Vector2D v2{1.f, 2.f};

  CHECK((v1 + v2).x == doctest::Approx(4.f));
  CHECK((v1 + v2).y == doctest::Approx(6.f));
  CHECK((v1 - v2).x == doctest::Approx(2.f));
  CHECK((v1 - v2).y == doctest::Approx(2.f));
  CHECK((v1 * 2.f).x == doctest::Approx(6.f));
  CHECK((v1 / 2.f).x == doctest::Approx(1.5f));

  CHECK(v1.norm2() == doctest::Approx(25.f));
  CHECK(v1.norm() == doctest::Approx(5.f));

  boids_sim::Vector2D v3 = v1.scale_to(10.f);
  CHECK(v3.norm() == doctest::Approx(10.f));
}
TEST_CASE("Vincoli Configurazione Stormo") {
  boids_sim::FlockConfiguration flock_config;
  flock_config.spawn_inf_edgeX_coeff = -0.5f;
  flock_config.spawn_sup_edgeX_coeff = 1.5f;
  flock_config.spawn_inf_edgeY_coeff = 0.5f;
  flock_config.spawn_sup_edgeY_coeff = 0.4f;
  flock_config.distV_amplitude = -5.f;
  flock_config.boids_num = -10;

  flock_config.constrain();

  CHECK(flock_config.spawn_inf_edgeX_coeff == doctest::Approx(0.01f));
  CHECK(flock_config.spawn_sup_edgeX_coeff == doctest::Approx(0.99f));
  CHECK(flock_config.spawn_inf_edgeY_coeff <=
        flock_config.spawn_sup_edgeY_coeff);
  CHECK(flock_config.distV_amplitude == doctest::Approx(0.f));
  CHECK(flock_config.boids_num == 0);
}
TEST_CASE("Passaggio dei Boid attraverso i bordi") {
  boids_sim::boid test_boid(995.f, 500.f, 10.f, 0.f);
  test_boid.update_position(1.f, 1000.f, 1000.f);
  CHECK(test_boid.getPosition().x == doctest::Approx(5.f));
  CHECK(test_boid.getPosition().y == doctest::Approx(500.f));
}
TEST_CASE("Componente UI Slider") {
  sf::Font font;
  REQUIRE(font.loadFromFile("arial.ttf"));
  boids_sim::slider test_slider(100.f, 100.f, 200.f, 0.f, 100.f, 50.f, font,
                                "Test Slider");
  SUBCASE("Inizializzazione Valore Default") {
    CHECK(test_slider.getValue() == doctest::Approx(50.f));
  }
  SUBCASE("Impostazione Valore e Clamping") {
    test_slider.setValue(25.f);
    CHECK(test_slider.getValue() == doctest::Approx(25.f));
    test_slider.setValue(-10.f);
    CHECK(test_slider.getValue() == doctest::Approx(0.f));
    test_slider.setValue(150.f);
    CHECK(test_slider.getValue() == doctest::Approx(100.f));
  }
  SUBCASE("Eccezione Costruttore Slider Min >= Max") {
    CHECK_THROWS(boids_sim::slider(100.f, 100.f, 200.f, 100.f, 50.f, 50.f, font,
                                   "Test"));
    CHECK_THROWS(boids_sim::slider(100.f, 100.f, 200.f, 100.f, 100.f, 100.f,
                                   font, "Test"));
  }
  SUBCASE("Eccezione Costruttore Slider Larghezza <= 0") {
    CHECK_THROWS(
        boids_sim::slider(100.f, 100.f, 0.f, 0.f, 100.f, 50.f, font, "Test"));
    CHECK_THROWS(
        boids_sim::slider(100.f, 100.f, -10.f, 0.f, 100.f, 50.f, font, "Test"));
  }
}
TEST_CASE("Test Predatore") {
  boids_sim::predator test_predator(0.f, 0.f, 0.f, 0.f);
  SUBCASE("Accelerazione Inseguimento e Movimento") {
    boids_sim::Vector2D target{100.f, 100.f};
    float vmax = 5.f;
    float accmax = 1.f;
    float ch = 1.f;
    float dt = 1.0f;
    float maxX = 1000.f;
    float maxY = 1000.f;
    boids_sim::Vector2D initial_pos = test_predator.getPosition();
    boids_sim::Vector2D dist = target.toroidal_minus(initial_pos, maxX, maxY);
    boids_sim::Vector2D chasing_acc =
        test_predator.calculate_chasing_acceleration(dist, vmax, accmax, ch);
    test_predator.update_velocity(chasing_acc, dt, vmax);
    test_predator.update_position(dt, maxX, maxY);
    boids_sim::Vector2D new_pos = test_predator.getPosition();
    boids_sim::Vector2D diff = new_pos.toroidal_minus(initial_pos, maxX, maxY);
    CHECK(diff.norm2() > 0.0001f);
  }
}
