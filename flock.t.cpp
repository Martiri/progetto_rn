#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
// ciao
#include "flock.hpp"

#include "SimValues.hpp"
#include "Vector2D.hpp"
#include "boid.hpp"
#include "doctest.h"

TEST_CASE("test di griglia") {
  boids_sim::flock myFlock(boids_sim::SimValues(
      500, 0.016f, 1.0f, 20.f, 2.f, 0.5f, 40.f, 5.f, 1600.f, 400.f, 20, 20,
      100.f, 50.f, 10000.f, 2500.f, 0.15f, 1200, 1));
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
    CHECK_THROWS(myFlock.getcell(pos, 10, -100.f));
  }
  SUBCASE("lenght zero") {
    boids_sim::Vector2D pos{50.f, 50.f};
    CHECK_THROWS(myFlock.getcell(pos, 10, 0.f));
  }
  /* SUBCASE("test neighbor cell wrapping") {
    CHECK(boids_sim::flock::get_neighbor_cell_index(9, 5, 1, 0, 10, 10) == 50);
    CHECK(boids_sim::flock::get_neighbor_cell_index(0, 5, -1, 0, 10, 10) == 59);
    CHECK(boids_sim::flock::get_neighbor_cell_index(5, 0, 0, -1, 10, 10) == 95);
    CHECK(boids_sim::flock::get_neighbor_cell_index(0, 0, -1, -1, 10, 10) ==
          99);
}
*/  
}