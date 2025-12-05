#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN //rega sta roba è solo un esempio di test che serve per far andare il cmake per come l'ho scritto se non da errore
#include "doctest.h"

#include "boid.hpp"
#include "flock.hpp"

using namespace boids_sim;

TEST_CASE("Vector2D operations") {
  Vector2D v1{3.f, 4.f};
  Vector2D v2{1.f, 2.f};

  CHECK(v1.norm2() == 25.f);

  Vector2D v3 = v1 + v2;
  CHECK(v3.x == 4.f);
  CHECK(v3.y == 6.f);

  Vector2D v4 = v1 - v2;
  CHECK(v4.x == 2.f);
  CHECK(v4.y == 2.f);

  Vector2D v5 = v1 * 2.f;
  CHECK(v5.x == 6.f);
  CHECK(v5.y == 8.f);
}

TEST_CASE("boid constructor and getters") {
  boid b{1.f, 2.f, 3.f, 4.f};

  CHECK(b.getPosition().x == 1.f);
  CHECK(b.getPosition().y == 2.f);
  CHECK(b.getVelocity().x == 3.f);
  CHECK(b.getVelocity().y == 4.f);
}

TEST_CASE("boid updatevelocity") {
  boid b{0.f, 0.f, 0.f, 0.f};
  b.updatevelocity({1.f, 2.f});

  CHECK(b.getVelocity().x == 1.f);
  CHECK(b.getVelocity().y == 2.f);
}

TEST_CASE("boid updateposition") {
  boid b{0.f, 0.f, 2.f, 3.f};
  b.updateposition(1.f);

  CHECK(b.getPosition().x == 2.f);
  CHECK(b.getPosition().y == 3.f);
}

TEST_CASE("flock constructor") {
  flock f{10, 400.f, 400.f};
  const auto &boids = f.getBoids();

  CHECK(boids.size() == 10);
}
