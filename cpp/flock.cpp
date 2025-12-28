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

flock::flock(int numBoids, float maxX, float maxY, int ncells)
    : numBoids_(numBoids), maxX_(maxX), maxY_(maxY) {
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
  headers_.resize(ncells, -1);
  next_.resize(numBoids_, -1);
  newvelocity_.resize(numBoids_);
}
const std::vector<boid> &flock::getBoids() const { return boids_; }

int flock::getcell(const Vector2D &position, const int &factorx,
                   const float &length) const {
  return static_cast<int>(position.y / length) * factorx +
         static_cast<int>(position.x / length);
}
void flock::step(float dt, float maxX, float maxY, int factorx, float s,
                 float a, float c, float d2, float ds2, float length,
                 int ncells, float vmax2) {
  std::fill(headers_.begin(), headers_.end(), -1);
  for (int i = 0; i < numBoids_; ++i) {
    int cell = getcell(boids_[i].getPosition(), factorx, length);
    next_[i] = headers_[cell];
    headers_[cell] = i;
  }
  for (int i = 0; i < numBoids_; ++i) {
    Vector2D accpos{0.f, 0.f};
    Vector2D accvel{0.f, 0.f};
    Vector2D accdist{0.f, 0.f};

    int l{0};
    int cell = getcell(boids_[i].getPosition(), factorx, length);
    for (int j = -1; j <= 1; j++)
      for (int k = -1; k <= 1; k++) {
        int act_cell{cell + j * factorx + k};
        if (act_cell >= 0 && act_cell < ncells) {
          int b = headers_[act_cell];
          while (b != -1) {
            const Vector2D &hposition = boids_[b].getPosition();
            const Vector2D &hvelocity = boids_[b].getVelocity();
            Vector2D dist = hposition - boids_[i].getPosition();
            float dist2 = dist.norm2();
            if (dist2 < d2) {
              l++;
              accpos += hposition;
              accvel += hvelocity;
              if (dist2 < ds2) accdist += dist;
            }
            b = next_[b];
          }
        }
      }
    if (l != 0) {
      Vector2D vs = accdist * (-s);
      Vector2D va = (accvel * (1.f / l) - boids_[i].getVelocity()) * a;
      Vector2D vc = (accpos * (1.f / l) - boids_[i].getPosition()) * c;
      newvelocity_[i] = vs + va + vc;
      boids_[i].updatevelocity(newvelocity_[i], vmax2);
      boids_[i].updateposition(dt, maxX, maxY);
    }
  }
}
};  // namespace boids_sim
