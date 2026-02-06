#include "slider.hpp"

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
  if (min >= max) {
    throw std::runtime_error("Slider min value must be less than max value.");
  }
  if (width <= 0.f) {
    throw std::runtime_error("Slider width must be positive.");
  }
  // Traccia (la linea di fondo)
  track.setSize(sf::Vector2f(width, 10));
  track.setFillColor(sf::Color(70, 100, 70, 200));
  track.setPosition(x, y);

  float percent = (defaultvalue - minValue) / (maxValue - minValue);
  float StartX = x + (percent * width);

  // Manopola (il cursore mobile)
  knob.setSize(sf::Vector2f(15, 25));
  knob.setFillColor(sf::Color(255, 255, 0, 210));
  knob.setOrigin(7.5f, 12.5f);
  knob.setPosition(StartX, y + 5);

  // Etichetta descrittiva
  label.setFont(font);
  label.setString(labelName);
  label.setCharacterSize(15);
  label.setStyle(sf::Text::Bold);
  label.setFillColor(sf::Color(255, 0, 150));
  label.setPosition(x, y - 22);

  // Valore numerico
  valueText.setFont(font);
  valueText.setCharacterSize(14);
  valueText.setStyle(sf::Text::Bold);
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

};  // namespace boids_sim