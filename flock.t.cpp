#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "boid.hpp"
#include "flock.hpp"

TEST_CASE("test di griglia") {
   boids_sim::flock myFlock(2, 1000.f, 1000.f, 100);
   SUBCASE("test getcell") {
     boids_sim::Vector2D pos1{50.f, 50.f};
     boids_sim::Vector2D pos2{950.f, 950.f};
     boids_sim::Vector2D pos3{7.5f, 92.3f};
     int cell1 = myFlock.getcell(pos1, 10, 100.f);
     int cell2 = myFlock.getcell(pos2, 10, 100.f);
     CHECK(cell1 == 0);
     CHECK(cell2 == 99);
     CHECK(myFlock.getcell(pos3, 10, 100.f) == 9);
   }
   SUBCASE("test bordi"){
    CHECK(myFlock.getcell(boids_sim::Vector2D{0.f, 0.f}, 10, 100.f) == 0);
    CHECK(myFlock.getcell(boids_sim::Vector2D{999.9f, 999.9f}, 10, 100.f) == 99);
   }
SUBCASE("lenght negativo") {
    boids_sim::Vector2D pos{50.f, 50.f};
    CHECK_THROWS(myFlock.getcell(pos, 10, -100.f));
}
SUBCASE("lenght zero") {
    boids_sim::Vector2D pos{50.f, 50.f};
    CHECK_THROWS(myFlock.getcell(pos, 10, 0.f));
    
}
}
TEST_CASE("nessun vicino"){
    boids_sim::flock myFlock(1, 1000.f, 1000.f, 100);
    myFlock.step(1000.f, 1000.f, 1000.f, 10, 1.5f, 0.5f, 0.01f, 10000.f, 25.f, 100.f, 100, 10000.f);
    CHECK(myFlock.getBoids()[0].getPosition().x == 0.f);
}


