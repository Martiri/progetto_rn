#include <SFML/Graphics.hpp>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace boids_sim {

class slider {
 public:
  sf::RectangleShape track;
  sf::RectangleShape knob;
  sf::Text label;
  sf::Text valueText;
  float minValue, maxValue;
  bool isDragging;
  slider(float x, float y, float width, float min, float max,
         float defaultvalue, sf::Font &font, std::string labelName);
  void handleEvent(sf::Event &event, sf::RenderWindow &window);
  void update(sf::RenderWindow &window);
  float getValue() const;
  void setValue(float value);
  void draw(sf::RenderWindow &window);
  void updateUI();
};
}  // namespace boids_sim

namespace boids_sim {

void slider::updateUI() {
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2) << getValue();
  valueText.setString(ss.str());
  valueText.setPosition(track.getPosition().x + track.getSize().x + 15,
                        track.getPosition().y - 5);
}
slider::slider(float x, float y, float width, float min, float max,
               float defaultvalue, sf::Font &font, std::string labelName)
    : minValue(min), maxValue(max), isDragging(false) {
  // Traccia (la linea di fondo)
  track.setSize(sf::Vector2f(width, 10));
  track.setFillColor(sf::Color(70, 70, 70));
  track.setPosition(x, y);

  float percent = (defaultvalue - minValue) / (maxValue - minValue);
  float StartX = x + (percent * width);

  // Manopola (il cursore mobile)
  knob.setSize(sf::Vector2f(15, 25));
  knob.setFillColor(sf::Color::White);
  knob.setOrigin(7.5f, 12.5f);
  knob.setPosition(StartX, y + 5);

  // Etichetta descrittiva
  label.setFont(font);
  label.setString(labelName);
  label.setCharacterSize(14);
  label.setFillColor(sf::Color(180, 180, 180));
  label.setPosition(x, y - 22);

  // Valore numerico
  valueText.setFont(font);
  valueText.setCharacterSize(14);
  valueText.setFillColor(sf::Color::Cyan);

  updateUI();
}

void slider::handleEvent(sf::Event &event, sf::RenderWindow &window) {
  sf::Vector2f mousePos =
      window.mapPixelToCoords(sf::Mouse::getPosition(window));

  if (event.type == sf::Event::MouseButtonPressed) {
    if (knob.getGlobalBounds().contains(mousePos)) {
      isDragging = true;
      knob.setFillColor(sf::Color(150, 150, 150));
    }
  }
  if (event.type == sf::Event::MouseButtonReleased) {
    isDragging = false;
    knob.setFillColor(sf::Color::White);
  }
}

void slider::update(sf::RenderWindow &window) {
  if (isDragging) {
    sf::Vector2f mousePos =
        window.mapPixelToCoords(sf::Mouse::getPosition(window));
    float newX = mousePos.x;

    // Clamping: mantieni la manopola dentro la traccia
    float trackLeft = track.getPosition().x;
    float trackRight = track.getPosition().x + track.getSize().x;

    if (newX < trackLeft) newX = trackLeft;
    if (newX > trackRight) newX = trackRight;

    knob.setPosition(newX, knob.getPosition().y);
    updateUI();
  }
}

float slider::getValue() const {
  float relativePos =
      (knob.getPosition().x - track.getPosition().x) / track.getSize().x;
  return minValue + relativePos * (maxValue - minValue);
}

void slider::setValue(float value) {
  if (value < minValue) value = minValue;
  if (value > maxValue) value = maxValue;

  float relativePos = (value - minValue) / (maxValue - minValue);
  float newX = track.getPosition().x + relativePos * track.getSize().x;
  knob.setPosition(newX, knob.getPosition().y);
  updateUI();
}

void slider::draw(sf::RenderWindow &window) {
  window.draw(track);
  window.draw(knob);
  window.draw(label);
  window.draw(valueText);
}

}  // namespace boids_sim

float brndf = 0.45;
int b_cost_rnd_time_magnitude = 1200;
int b_var_rnd_time_magnitude = b_cost_rnd_time_magnitude + 1;
float velafp = 0.6f;
float velafm = 0.95f;
float fatiguef = -0.6f;
float rushf = 0.5f;
namespace boids_sim {
struct Vector2D {
  float x;
  float y;

  Vector2D operator+(const Vector2D &other) const;
  Vector2D operator-(const Vector2D &other) const;
  Vector2D operator*(float scalar) const;
  Vector2D &operator+=(const Vector2D &other);
  Vector2D &operator*=(const float scalar);
  float norm2() const;
  float dot(const Vector2D &other) const;
};
class boid {
 private:
  Vector2D position_{0.f, 0.f};
  Vector2D velocity_{0.f, 0.f};
  Vector2D acceleration_{0.f, 0.f};
  Vector2D random_acceleration_{0.f, 0.f};

  int random_timer_{0};
  int rush_{0};
  int fatigue_{0};

 public:
  boid() = default;
  boid(float x, float y, float vx, float vy, float ax, float ay, float rnd_accx,
       float rnd_accy, int rnd_timer);

  const Vector2D &getPosition() const;
  const Vector2D &getVelocity() const;
  const Vector2D &getAcceleration() const;
  // void velocityalignmentacceleration(float idealv2);
  void updateposition(float dt, float maxX, float maxY);
  void updatevelocity(float dt, float vmax, float vmax2);
  void updateacceleration(const Vector2D &acc, float accmax, float accmax2,
                          const Vector2D &swerve_acc,
                          const Vector2D &fatigue_acc,
                          const Vector2D &rush_acc);
  void updaterandombehaviour(float accmax);
  Vector2D updateswerveacceleration();
  Vector2D updatefatigueacceleration(const float stamina,
                                     const float fatigue_threshold_v2) const;
  Vector2D updaterushacceleration(const float patience,
                                  const float rush_threshold_v2) const;
  void updatefatigue(float fatigue_threshold_v2);
  void updaterush(float rush_threshold_v2, float comeback_threshold_v2);
};
}  // namespace boids_sim

namespace boids_sim {
class flock {
 private:
  std::vector<boid> boids_;
  int numBoids_;
  float maxX_;
  float maxY_;
  std::vector<int> headers_;
  std::vector<int> next_;
  std::vector<Vector2D> newaccelerations_;
  float idealv2_;
  float fatigue_threshold_v2_;
  float rush_threshold_v2_;
  float comeback_threshold_v2_;
  float stamina_;
  float patience_;

 public:
  flock(int numBoids, float maxX, float maxY, int ncells, float idealv,
        float fatigue_threshold_v, float rush_threshold_v,
        float comeback_threshold_v);
  int getcell(const Vector2D &position, const int &factorx,
              const float &length) const;
  int getXcoord(const Vector2D &position, const float &length) const;
  int getYcoord(const Vector2D &position, const float &length) const;
  void step(float dt, float maxX, float maxY, int factorx, int factory, float s,
            float a, float c, float d2, float ds2, float length, float vmax,
            float vmax2, float accmax, float accmax2);
  const std::vector<boid> &getBoids() const;
  void reset_headers() {}
  Vector2D computeAverageVelocity() const;
  float computeAverageDistance() const;
  Vector2D b_toroidaldistance(const Vector2D &hpos, const boid &b) const;
};
};  // namespace boids_sim
namespace boids_sim {
Vector2D Vector2D::operator+(const Vector2D &other) const {
  return {x + other.x, y + other.y};
}
Vector2D Vector2D::operator-(const Vector2D &other) const {
  return {x - other.x, y - other.y};
}
Vector2D Vector2D::operator*(float scalar) const {
  return {x * scalar, y * scalar};
}
Vector2D &Vector2D::operator+=(const Vector2D &other) {
  x += other.x;
  y += other.y;
  return *this;
}
Vector2D &Vector2D::operator*=(const float scalar) {
  x *= scalar;
  y *= scalar;
  return *this;
}

float Vector2D::norm2() const { return x * x + y * y; }
float Vector2D::dot(const Vector2D &other) const {
  return x * other.x + y * other.y;
}
boid::boid(float x, float y, float vx, float vy, float ax, float ay,
           float rnd_accx, float rnd_accy, int rnd_timer)
    : position_{x, y},
      velocity_{vx, vy},
      acceleration_{ax, ay},
      random_acceleration_{rnd_accx, rnd_accy},
      random_timer_{rnd_timer},
      rush_{0},
      fatigue_{0} {};
const Vector2D &boid::getPosition() const { return position_; }
const Vector2D &boid::getVelocity() const { return velocity_; }
const Vector2D &boid::getAcceleration() const { return acceleration_; }
void boid::updaterandombehaviour(float accmax) {
  if (random_timer_ <= 0) {
    float acc_magnitude = brndf * accmax;
    random_acceleration_ = {
        (static_cast<float>(rand()) / RAND_MAX * 2.f - 1.f) * acc_magnitude,
        (static_cast<float>(rand()) / RAND_MAX * 2.f - 1.f) * acc_magnitude};
    random_timer_ =
        b_cost_rnd_time_magnitude + rand() % b_var_rnd_time_magnitude;
  }
  random_timer_--;
}
Vector2D boid::updateswerveacceleration() {
  if (velocity_.dot(acceleration_) <
      0.5 * std::sqrt(acceleration_.norm2()) *
          std::sqrt(velocity_.norm2()))  // || velocity_.norm2() < idealv2) {
    return acceleration_ * velafp;
  else
    return {0.f, 0.f};
}
Vector2D boid::updatefatigueacceleration(
    const float stamina, const float fatigue_threshold_v2) const {
  if (fatigue_ >= stamina && velocity_.norm2() > fatigue_threshold_v2)
    return velocity_ * fatiguef;
  else
    return {0.f, 0.f};
}
Vector2D boid::updaterushacceleration(const float patience,
                                      const float comeback_threshold_v2) const {
  if (rush_ >= patience && velocity_.norm2() < comeback_threshold_v2)
    return velocity_ * rushf;
  else
    return {0.f, 0.f};
}
void boid::updateacceleration(const Vector2D &acc, float accmax, float accmax2,
                              const Vector2D &swerve_acc,
                              const Vector2D &fatigue_acc,
                              const Vector2D &rush_acc) {
  float acc_norm2 = acc.norm2();
  if (acc_norm2 <= accmax2) {
    acceleration_ = acc;
  } else {
    acceleration_ = acc * (1.f / std::sqrt(acc_norm2)) * accmax;
  }
  acceleration_ += random_acceleration_;
  acceleration_ += swerve_acc;
  acceleration_ += fatigue_acc;
  acceleration_ += rush_acc;
}
void boid::updatevelocity(float dt, float vmax, float vmax2) {
  Vector2D v_new = velocity_ + acceleration_ * dt;
  float v_new_norm2 = v_new.norm2();
  if (v_new_norm2 <= vmax2) {
    velocity_ = v_new;
  } else {
    velocity_ = v_new * (1.f / std::sqrt(v_new_norm2)) * vmax;
  }
}
void boid::updateposition(float dt, float maxX, float maxY) {
  position_ += velocity_ * dt;
  if (position_.x < 0) {
    position_.x += maxX;
  } else if (position_.x > maxX) {
    position_.x -= maxX;
  }
  if (position_.y < 0) {
    position_.y += maxY;
  } else if (position_.y > maxY) {
    position_.y -= maxY;
  }
}
void boid::updatefatigue(float fatigue_threshold_v2) {
  if (velocity_.norm2() > fatigue_threshold_v2)
    fatigue_++;
  else if (fatigue_ > 0)
    fatigue_--;
}
void boid::updaterush(float rush_threshold_v2, float comeback_threshold_v2) {
  if (velocity_.norm2() < rush_threshold_v2)
    rush_++;
  else if (velocity_.norm2() > comeback_threshold_v2 && rush_ > 0)
    rush_--;
}
};  // namespace boids_sim
float hamplitudev = 200.f;
float offsetv = 70.f;
float fact = 0.5f;
float inf_spawnfX = 0.5f - fact * (1.f / (2 * std::sqrt(2)));
float sup_spawnfX = 0.5f + fact * (1.f / (2 * std::sqrt(2)));
float inf_spawnfY = inf_spawnfX;
float sup_spawnfY = sup_spawnfX;
float s;
float a;
float c;
float d;
float d2 = d * d;
float ds;
float ds2 = ds * ds;
float timescale;
float vmax = 250.f;
float idealv = vmax * 0.5;
float staminaf = 0.5f;
float lazinessf = 0.3f;
float comebackf = 0.8f;
float fatigue_threshold_v = vmax * staminaf;
float rush_threshold_v = vmax * lazinessf;
float comeback_threshold_v = vmax * comebackf;
int stamina = 200;
int patience = 50;
namespace boids_sim {

flock::flock(int numBoids, float maxX, float maxY, int ncells, float idealv,
             float fatigue_threshold_v, float rush_threshold_v,
             float comeback_threshold_v)
    : numBoids_(numBoids),
      maxX_(maxX),
      maxY_(maxY),
      idealv2_(idealv * idealv),
      fatigue_threshold_v2_(fatigue_threshold_v * fatigue_threshold_v),
      rush_threshold_v2_(rush_threshold_v * rush_threshold_v),
      comeback_threshold_v2_(comeback_threshold_v * comeback_threshold_v),
      stamina_(stamina),
      patience_(patience) {
  boids_.clear();
  boids_.reserve(static_cast<size_t>(numBoids_));

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> distX(maxX_ * inf_spawnfX,
                                              maxX_ * sup_spawnfX);
  std::uniform_real_distribution<float> distY(maxY_ * inf_spawnfY,
                                              maxY_ * sup_spawnfY);
  std::uniform_real_distribution<float> distV(-hamplitudev + offsetv,
                                              hamplitudev + offsetv);

  for (int i = 0; i < numBoids_; ++i) {
    boids_.emplace_back(distX(gen), distY(gen), distV(gen), distV(gen), 0.f,
                        0.f, 0.f, 0.f, 0);
  }
  headers_.resize(ncells, -1);
  next_.resize(numBoids_, -1);
  newaccelerations_.resize(numBoids_);
}
const std::vector<boid> &flock::getBoids() const { return boids_; }
Vector2D flock::b_toroidaldistance(const Vector2D &hpos, const boid &b) const {
  Vector2D dist = hpos - b.getPosition();
  if (dist.x > maxX_ * 0.5f)
    dist.x -= maxX_;
  else if (dist.x < -maxX_ * 0.5f)
    dist.x += maxX_;
  if (dist.y > maxY_ * 0.5f)
    dist.y -= maxY_;
  else if (dist.y < -maxY_ * 0.5f)
    dist.y += maxY_;
  return dist;
}
int flock::getcell(const Vector2D &position, const int &factorx,
                   const float &length) const {
  return static_cast<int>(position.y / length) * factorx +
         static_cast<int>(position.x / length);
}
int flock::getXcoord(const Vector2D &position, const float &length) const {
  return static_cast<int>(position.x / length);
}
int flock::getYcoord(const Vector2D &position, const float &length) const {
  return static_cast<int>(position.y / length);
}
void flock::step(float dt, float maxX, float maxY, int factorx, int factory,
                 float s, float a, float c, float d2, float ds2, float length,
                 float vmax, float vmax2, float accmax, float accmax2) {
  std::fill(headers_.begin(), headers_.end(), -1);
  for (int i = 0; i < numBoids_; ++i) {
    // int cell = getcell(boids_[i].getPosition(), factorx, length);
    int cx = getXcoord(boids_[i].getPosition(), length);
    int cy = getYcoord(boids_[i].getPosition(), length);
    int cell = cy * factorx + cx;
    next_[i] = headers_[cell];
    headers_[cell] = i;
  }
  for (int i = 0; i < numBoids_; ++i) {
    // Vector2D cumpos{0.f, 0.f};
    Vector2D cumvel{0.f, 0.f};
    Vector2D cumdist{0.f, 0.f};
    Vector2D cumshortdist{0.f, 0.f};
    int l{0};
    // int cell = getcell(boids_[i].getPosition(), factorx, length);
    int cx = getXcoord(boids_[i].getPosition(), length);
    int cy = getYcoord(boids_[i].getPosition(), length);
    for (int j = -1; j <= 1; j++)
      for (int k = -1; k <= 1; k++) {
        int act_cell{((cx + k + factorx) % factorx) +
                     ((cy + j + factory) % factory) * factorx};
        // if (act_cell >= 0 && act_cell < ncells) {
        int b = headers_[act_cell];
        while (b != -1) {
          const Vector2D &hposition = boids_[b].getPosition();
          const Vector2D &hvelocity = boids_[b].getVelocity();
          // Vector2D dist = hposition - boids_[i].getPosition();
          Vector2D dist = b_toroidaldistance(hposition, boids_[i]);
          float dist2 = dist.norm2();
          Vector2D velocity = boids_[i].getVelocity();
          if (dist2 < d2 &&
              velocity.dot(dist) >
                  -0.5f * std::sqrt(velocity.norm2()) * std::sqrt(dist2)) {
            l++;
            // cumpos += hposition;
            cumdist += dist;
            cumvel += hvelocity;
            if (dist2 < ds2) cumshortdist += dist;
          }
          b = next_[b];
        }
        // }
      }
    if (l != 0) {
      Vector2D accs = cumshortdist * (-s);
      Vector2D acca = (cumvel * (1.f / l) - boids_[i].getVelocity()) * a;
      // Vector2D accc = (cumpos * (1.f / l) - boids_[i].getPosition()) * c;
      Vector2D accc = cumdist * (1.f / l) * c;
      newaccelerations_[i] = accs + acca + accc;
    }
  }
  for (int i = 0; i < numBoids_; i++) {
    boids_[i].updaterandombehaviour(accmax);
    boids_[i].updateacceleration(
        newaccelerations_[i], accmax, accmax2,
        boids_[i].updateswerveacceleration(),
        boids_[i].updatefatigueacceleration(stamina_, fatigue_threshold_v2_),
        boids_[i].updaterushacceleration(patience_, rush_threshold_v2_));
    boids_[i].updatevelocity(dt, vmax, vmax2);
    boids_[i].updateposition(dt, maxX, maxY);
    boids_[i].updatefatigue(fatigue_threshold_v2_);
    boids_[i].updaterush(rush_threshold_v2_, comeback_threshold_v2_);
  }
}
};  // namespace boids_sim

int main() {
  try {
    int numBoids = 400;
    // float f = 4.f;
    float s = 5.0f;
    float a = 1.5f;
    float c = 1.f;
    float d = 100.0f;
    float ds = 5.0f;
    float length = d;
    float d2 = d * d;
    float ds2 = ds * ds;
    int factorx = 9;
    int factory = 9;
    int ncells = factorx * factory;
    float maxX = length * factorx;
    float maxY = length * factory;
    float dt = 0.016f;
    float timescale = 1.0f;
    float vmax2 = vmax * vmax;
    float accmax = 120.f;
    float accmax2 = accmax * accmax;

    boids_sim::flock flock(numBoids, maxX, maxY, ncells, idealv,
                           fatigue_threshold_v, rush_threshold_v,
                           comeback_threshold_v);
    // float idealv2 = idealv * idealv;
    sf::RenderWindow window(sf::VideoMode(maxX, maxY), "Boids Simulation");
    sf::Font font;
    if (!font.loadFromFile("arial.ttf")) {
      throw std::runtime_error("Impossibile caricare il font arial.ttf");
    }
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
                         vmax * 2, vmax, font, "vmax");
    sliders.back().setValue(vmax);
    sf::CircleShape boidShape(4.0f);
    boidShape.setFillColor(sf::Color::Blue);
    boidShape.setOrigin(4.0f, 4.0f);

    sf::Clock clock;

    while (window.isOpen()) {
      sf::Event event;
      while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) window.close();
        for (auto &s : sliders) s.handleEvent(event, window);
      }

      dt = clock.restart().asSeconds();
      if (dt > 0.1) dt = 0.1;
      dt *= timescale;

      for (auto &s : sliders) s.update(window);

      s = sliders[0].getValue();
      a = sliders[1].getValue();
      c = sliders[2].getValue();
      d = sliders[3].getValue();
      ds = sliders[4].getValue();
      vmax = sliders[5].getValue();

      vmax2 = vmax * vmax;
      d2 = d * d;
      ds2 = ds * ds;
      flock.step(dt, maxX, maxY, factorx, factory, s, a, c, d2, ds2, length,
                 vmax, vmax2, accmax, accmax2);
      window.clear(sf::Color::Black);

      for (const auto &boid : flock.getBoids()) {
        const auto &pos = boid.getPosition();
        boidShape.setPosition(static_cast<float>(pos.x),
                              static_cast<float>(pos.y));
        window.draw(boidShape);
        // std::cout << "vel :" << boid.getVelocity().x << ", " <<
        // boid.getVelocity().y << std::endl;
      }

      for (auto &s : sliders) s.draw(window);

      window.display();
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
