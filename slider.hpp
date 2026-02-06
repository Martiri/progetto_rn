#ifndef SLIDER_HPP
#define SLIDER_HPP
#include <SFML/Graphics.hpp>
#include <string>

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
         float defaultvalue, const sf::Font &font, const std::string &labelName);
  void handleEvent(const sf::Event &event, const sf::RenderWindow &window);
  void update(const sf::RenderWindow &window);
  float getValue() const;
  void setValue(float value);
  void draw(sf::RenderWindow &window);
  void updateUI();
};
};  // namespace boids_sim
#endif