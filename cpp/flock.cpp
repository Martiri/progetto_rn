#include "flock.hpp"

#include <cassert>
#include <random>
float hamplitudev = 100.f;
float offsetv = 4.f;
float s = 0.15f;
float a = 0.08f;
float c = 0.01f;
float d = 60.0f;
float d2 = d * d;
float ds = 20.0f;
float ds2 = ds * ds;
float timescale = 1.0f;
namespace boids_sim {

flock::flock(int numBoids, float maxX, float maxY)
    : numBoids_(numBoids), maxX_(maxX), maxY_(maxY) {
  boids_.clear();
  boids_.reserve(static_cast<size_t>(numBoids_));

  if (factorx > 0) l = maxX_ / factorx;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> distX(0.0f, maxX_);
  std::uniform_real_distribution<float> distY(0.0f, maxY_);
  std::uniform_real_distribution<float> distV(-hamplitudev + offsetv,
                                              hamplitudev + offsetv);

  for (int i = 0; i < numBoids_; ++i) {
    boids_.emplace_back(distX(gen), distY(gen), distV(gen), distV(gen));
  }
  headers_.resize(static_cast<size_t>(ncells), -1);
  next_.resize(static_cast<size_t>(numBoids_), -1);
  newvelocity_.resize(static_cast<size_t>(numBoids_));
}
const std::vector<boid>& flock::getBoids() const { return boids_; }

int flock::getcell(const Vector2D& position) const {
  return static_cast<int>(position.y / l) * factorx +
         static_cast<int>(position.x / l);
}
void flock::step(float dt, float factorx, float s, float a, float c, float d,
                 float ds) {
  for (int i = 0; i < numBoids_; ++i) {
    int cell = getcell(boids_[i].getPosition());
    if (cell >= 0 && cell < ncells) {
      if (headers_[static_cast<size_t>(cell)] != -1) {
        next_[static_cast<size_t>(i)] = headers_[static_cast<size_t>(cell)];
      }
      headers_[static_cast<size_t>(cell)] = i;
    }
  }
  for (int i = 0; i < numBoids_; ++i) {
    Vector2D accpos{0.f, 0.f};
    Vector2D accvel{0.f, 0.f};
    Vector2D accdist{0.f, 0.f};

    int neighbor_count{0};
    int cell = getcell(boids_[i].getPosition());

    float local_d2 = d * d;
    float local_ds2 = ds * ds;

    for (int dy = -1; dy <= 1; dy++)
      for (int dx = -1; dx <= 1; dx++) {
        int neighbor_cell = cell + dy * static_cast<int>(factorx) + dx;
        if (neighbor_cell < 0 || neighbor_cell >= ncells) continue;
        int b = headers_[static_cast<size_t>(neighbor_cell)];
        while (b != -1) {
          const Vector2D& hposition = boids_[static_cast<size_t>(b)].getPosition();
          const Vector2D& hvelocity = boids_[static_cast<size_t>(b)].getVelocity();
          Vector2D dist = hposition - boids_[static_cast<size_t>(i)].getPosition();
          float dist2 = dist.norm2();
          if (dist2 < local_d2) {
            neighbor_count += 1;
            accpos += hposition;
            accvel += hvelocity;
            if (dist2 < local_ds2) accdist += dist;
          }
          b = next_[static_cast<size_t>(b)];
        }
      }

    Vector2D vs = accdist * (-s);
    Vector2D va{0.f, 0.f};
    Vector2D vc{0.f, 0.f};

    if (neighbor_count > 0) {
        va = (accvel * (1.0f / neighbor_count) - boids_[static_cast<size_t>(i)].getVelocity()) * a;
        vc = (accpos * (1.0f / neighbor_count) - boids_[static_cast<size_t>(i)].getPosition()) * c;
    }

    newvelocity_[static_cast<size_t>(i)] = vs + va + vc;
    boids_[static_cast<size_t>(i)].updatevelocity(newvelocity_[static_cast<size_t>(i)]);
    boids_[static_cast<size_t>(i)].updateposition(dt);
  }
}
}  // namespace boids_sim