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
    : numBoids_(numBoids), maxX_(maxX), maxY_(maxY),
      factorx_(20), factory_(20) {
        if (factorx_ > 0)  l_ = maxX_ / factorx_;
  ncells_ = factorx_ * factory_;
  boids_.clear();
  boids_.reserve(static_cast<size_t>(numBoids_));

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> distX(0.0f, maxX_);
  std::uniform_real_distribution<float> distY(0.0f, maxY_);
  std::uniform_real_distribution<float> distV(-hamplitudev + offsetv,
                                              hamplitudev + offsetv);

  for (int i = 0; i < numBoids_; ++i) {
    boids_.emplace_back(distX(gen), distY(gen), distV(gen), distV(gen));
  }
  headers_.resize(ncells_, -1);
  next_.resize(numBoids_, -1);
  newvelocity_.resize(numBoids_);
}
const std::vector<boid>& flock::getBoids() const { return boids_; }

int flock::getcell(const Vector2D& position) const {
  return static_cast<int>(position.y / l_) * factorx_ +
         static_cast<int>(position.x / l_);
}
void flock::step(float dt, float factorx, float s, float a, float c, float d,
                 float ds) {
  for (int i = 0; i < numBoids_; ++i) {
    int cell = getcell(boids_[i].getPosition());
    if (headers_[cell] != -1) {
      next_[i] = headers_[cell];
    }

    headers_[cell] = i;
  }
  for (int i = 0; i < numBoids_; ++i) {
    Vector2D accpos{0.f, 0.f};
    Vector2D accvel{0.f, 0.f};
    Vector2D accdist{0.f, 0.f};

    int l{0};
    int cell = getcell(boids_[i].getPosition());
    for (int i = -1; i <= 1; i++)
      for (int j = -1; j <= 1; j++) {
        int b = headers_[cell + i * static_cast<int>(factorx_) + j];
        while (b != -1) {
          const Vector2D& hposition = boids_[b].getPosition();
          const Vector2D& hvelocity = boids_[b].getVelocity();
          Vector2D dist = hposition - boids_[i].getPosition();
          float dist2 = dist.norm2();
          if (dist2 < d2) {
            l += 1;
            accpos += hposition;
            accvel += hvelocity;
            if (dist2 < ds2) accdist += dist;
          }
          b = next_[b];
        }
      }
    Vector2D vs = accdist * (-s);
    Vector2D va = (accvel * (1 / l) - boids_[i].getVelocity()) * a;
    Vector2D vc = (accpos * (1 / l) - boids_[i].getPosition()) * c;
    newvelocity_[i] = vs + va + vc;
    boids_[i].updatevelocity(newvelocity_[i]);
    boids_[i].updateposition(dt);
  }
}
}  // namespace boids_sim