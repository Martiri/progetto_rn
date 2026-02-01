#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <SFML/Graphics.hpp>
#include <vector>

#include "flock.hpp"
#include "predator.hpp"
#include "simvalues.hpp"
#include "Vector2D.hpp"
#include "boid.hpp"
#include "slider.hpp"
#include "doctest.h"

TEST_CASE("test di griglia") {
  boids_sim::SimValues sim_values;
  sim_values.boids_num = 500;
  sim_values.maxX = 999.f; 
  sim_values.maxY = 999.f;
  sim_values.spawn_inf_edgeX_coeff = 0.f;
  sim_values.spawn_sup_edgeX_coeff = 1.f;
  sim_values.spawn_inf_edgeY_coeff = 0.f;
  sim_values.spawn_sup_edgeY_coeff = 1.f;
  sim_values.factorx = 10;
  sim_values.factory = 10;
  sim_values.ncells = sim_values.factorx * sim_values.factory;
  sim_values.distV_amplitude = 2.f;
  sim_values.distV_offset = 0.f;
  
  boids_sim::flock myFlock(sim_values);
  SUBCASE("test getcell") {
    boids_sim::Vector2D pos1{50.f, 50.f};
    boids_sim::Vector2D pos2{950.f, 950.f};
    boids_sim::Vector2D pos3{7.5f, 92.3f};
    int cell1 = myFlock.getcell(pos1, 10, 100.f);
    int cell2 = myFlock.getcell(pos2, 10, 100.f);
    CHECK(cell1 == 0);
    CHECK(cell2 == 99);
    CHECK(myFlock.getcell(pos3, 10, 100.f) == 0);
  }
  SUBCASE("test bordi") {
    CHECK(myFlock.getcell(boids_sim::Vector2D{0.f, 0.f}, 10, 100.f) == 0);
    CHECK(myFlock.getcell(boids_sim::Vector2D{999.9f, 999.9f}, 10, 100.f) ==
          99);
  }
  SUBCASE("length negativo") {
    boids_sim::Vector2D pos{50.f, 50.f};
    CHECK_NOTHROW(myFlock.getcell(pos, 10, -100.f));
  }
  SUBCASE("lenght zero") {
    boids_sim::Vector2D pos{50.f, 50.f};
    CHECK_NOTHROW(myFlock.getcell(pos, 10, 1.f));
  }
  SUBCASE("l negativo") {
    CHECK(myFlock.get_boids_num() == static_cast<int>(sim_values.boids_num));
  }
  SUBCASE("test b_toroidaldistance") {
    boids_sim::boid testBoid(100.f, 100.f, 0.f, 0.f);
    boids_sim::Vector2D hpos1{150.f, 150.f};
    auto b_toroidaldistance = [](const boids_sim::Vector2D &hpos,
                                  const boids_sim::boid &b, float maxX,
                                  float maxY) {
      boids_sim::Vector2D d;
      float dx = hpos.x - b.getPosition().x;
      if (dx > maxX / 2.f)
        dx -= maxX;
      else if (dx < -maxX / 2.f)
        dx += maxX;
      float dy = hpos.y - b.getPosition().y;
      if (dy > maxY / 2.f)
        dy -= maxY;
      else if (dy < -maxY / 2.f)
        dy += maxY;
      d.x = dx;
      d.y = dy;
      return d;
    };
    boids_sim::Vector2D dist1 = b_toroidaldistance(hpos1, testBoid, sim_values.maxX, sim_values.maxY);
    CHECK(dist1.x == doctest::Approx(50.f));
    CHECK(dist1.y == doctest::Approx(50.f));
    boids_sim::Vector2D hpos2{950.f, 100.f}; 
    boids_sim::Vector2D dist2 = b_toroidaldistance(hpos2, testBoid, sim_values.maxX, sim_values.maxY);
    CHECK(dist2.x == doctest::Approx(-149.f)); 
    CHECK(dist2.y == doctest::Approx(0.f));
    boids_sim::Vector2D hpos3{100.f, 950.f};
    boids_sim::Vector2D dist3 = b_toroidaldistance(hpos3, testBoid, sim_values.maxX, sim_values.maxY);
    CHECK(dist3.x == doctest::Approx(0.f));
    CHECK(dist3.y == doctest::Approx(-149.f));
  }
  SUBCASE("test step execution") {
    sim_values.d = 100.f;
    sim_values.ds = 50.f;
    sim_values.d2 = sim_values.d * sim_values.d;
    sim_values.ds2 = sim_values.ds * sim_values.ds;
    sim_values.factorx = 10;
    sim_values.factory = 10;
    sim_values.ncells = sim_values.factorx * sim_values.factory;
    sim_values.vmax = 5.f;
    sim_values.accmax = 1.f;
    sim_values.vmax2 = sim_values.vmax * sim_values.vmax;
    sim_values.accmax2 = sim_values.accmax * sim_values.accmax;
    sim_values.s = 1.f;
    sim_values.a = 1.f;
    sim_values.c = 1.f;
    sim_values.e = 1.f;
    sim_values.ch = 1.f;
    sim_values.escape_d = 200.f;
    sim_values.escape_d2 = sim_values.escape_d * sim_values.escape_d;
    sim_values.predator_vmax = 5.f;
    sim_values.predator_vmax2 = sim_values.predator_vmax * sim_values.predator_vmax;
    sim_values.predator_d = 150.f;
    sim_values.predator_d2 = sim_values.predator_d * sim_values.predator_d;
    sim_values.dt = 0.016f;
    sim_values.distV_amplitude = 2.f;
    sim_values.distV_offset = 0.f;
    std::vector<boids_sim::Vector2D> initial_velocities;
    std::vector<boids_sim::Vector2D> initial_positions;
    for (const auto& b : myFlock.getBoids()) {
      initial_velocities.push_back(b.getVelocity());
      initial_positions.push_back(b.getPosition());
    }
    myFlock.step(sim_values);
    const auto& boids_after = myFlock.getBoids();
    bool velocities_changed = false;
    bool positions_changed = false;
    for (size_t i = 0; i < initial_velocities.size(); ++i) {
      boids_sim::Vector2D vel_diff = boids_after[i].getVelocity() - initial_velocities[i];
      if (vel_diff.norm2() > 0.0001f) {
        velocities_changed = true;
      }
      boids_sim::Vector2D pos_diff = boids_after[i].getPosition() - initial_positions[i];
      if (pos_diff.norm2() > 0.0001f) {
        positions_changed = true;
      }
    }
    CHECK(velocities_changed); 
    CHECK(positions_changed); 
  }
  
}
TEST_CASE("test slider") {
  sf::Font font;
  font.loadFromFile("arial.ttf");
  boids_sim::slider mySlider(100.f, 100.f, 200.f, 0.f, 100.f, 50.f, font, "Test Slider");
  SUBCASE("test getValue default") {
    CHECK(mySlider.getValue() == doctest::Approx(50.f));
  }
  SUBCASE("test setValue") {
    mySlider.setValue(25.f);
    CHECK(mySlider.getValue() == doctest::Approx(25.f));
    mySlider.setValue(-10.f);
    CHECK(mySlider.getValue() == doctest::Approx(0.f));
    mySlider.setValue(150.f); 
    CHECK(mySlider.getValue() == doctest::Approx(100.f));
  }
}
TEST_CASE("test predator") {
  boids_sim::predator pred(0.f, 0.f, 0.f, 0.f); 
  SUBCASE("test updatechasingacceleration and movement") {
    boids_sim::Vector2D target{100.f, 100.f};
    float vmax = 5.f;
    float accmax = 1.f;
    float ch = 1.f;
    float dt = 1.0f; 
    float maxX = 1000.f;
    float maxY = 1000.f;
    boids_sim::Vector2D initial_pos = pred.getPosition();
    pred.updatechasingacceleration(target, vmax, accmax, ch);
    pred.updatevelocity(dt, vmax);
    pred.updateposition(dt, maxX, maxY);
    boids_sim::Vector2D new_pos = pred.getPosition();
    boids_sim::Vector2D diff = new_pos - initial_pos;
    CHECK(diff.norm2() > 0.0001f); 
  }
  SUBCASE("test resetacceleration") {
    pred.resetacceleration();
    CHECK(true); 
  }
}