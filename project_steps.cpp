/* 1) trovare le immagini (preferibilmente quadrate)
   2) adattarle alla giusta dimensione (interpolazione bilineare)
   INTERPOLAZIONE
   per ogni pixel si calcola la posizione virtuale (Xv, Yv)
   per ogni pixel si fa una media dei pixel che hanno posizione virtuale in
   quella zona*/

#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <numeric>
int main() {
  sf::Image colored_scarlett_image;
  colored_scarlett_image.loadFromFile("orecchio.png");
  int old_w{colored_scarlett_image.getSize().x};
  int old_h{colored_scarlett_image.getSize().y};
  float w{64}, h{64};
  float w_conv{old_w / w}, h_conv{old_h / h};
  std::vector<sf::Color> colored_scarlett_pixels(old_w * old_h);
  for (unsigned int x{}; x < old_w; x++) {
    for (unsigned int y{}; y < old_h; y++) {
      colored_scarlett_pixels[y * old_w + x] = colored_scarlett_image.getPixel(x, y);
    }
  }
  sf::Image adapted_image;
  adapted_image.create(w, h);
  std::vector<std::vector<sf::Color>> packed_new_pixels(w*h);
  for (unsigned int x{}; x < old_w; x++) {
    for (unsigned int y{}; y < old_h; y++) {
      float x_v = x/w_conv; 
      float y_v = y/h_conv;
      // I have to recognize the new_pixel block of belonging of the old_pixel
    int x_pack{std::min((int) x_v, (int) w-1)};
    int y_pack{std::min((int) y_v, (int) h-1)};
    packed_new_pixels[y_pack*w+x_pack].push_back(colored_scarlett_pixels[y*old_w+x]);
    }
  }
  int i{0};
  for ( std::vector <sf::Color> pixel_pack : packed_new_pixels){
    float avg_r = std::accumulate(pixel_pack.begin(), pixel_pack.end(), 0.f,
[](float sum, const sf::Color &pixel) {return sum + pixel.r;}) / pixel_pack.size() ;
    float avg_g = std::accumulate(pixel_pack.begin(), pixel_pack.end(), 0.f,
[](float sum, const sf::Color &pixel) {return sum + pixel.g;}) / pixel_pack.size() ;
    float avg_b = std::accumulate(pixel_pack.begin(), pixel_pack.end(), 0.f,
[](float sum, const sf::Color &pixel) {return sum + pixel.b;}) / pixel_pack.size() ;
    adapted_image.setPixel(i%(int)w , i/w, sf::Color(avg_r, avg_g, avg_b));
    i++;
  }
sf::Texture adapted_image_texture;
  adapted_image_texture.loadFromImage(adapted_image);
  sf::Sprite adapted_image_sprite(adapted_image_texture);
  sf::RenderWindow adapted_image_window(sf::VideoMode(w, h),
                                         "scarlett_image_adapted");                                       
  adapted_image_window.clear();
  adapted_image_window.draw(adapted_image_sprite);
  adapted_image_window.display();
  while (adapted_image_window.isOpen()) {
    sf::Event event;
    while (adapted_image_window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        adapted_image_window.close();
      }
    }
  }


  std::cout <<(int) colored_scarlett_pixels[0].r << std::endl;
  std::cout << (int) colored_scarlett_pixels[120000].r << std::endl;
  std::cout << (int) colored_scarlett_pixels[180000].r << std::endl;
  std::cout << (int) colored_scarlett_pixels[62000].r << std::endl;
  std::cout << (int) colored_scarlett_pixels[std::size(colored_scarlett_pixels) - 1].r
            << std::endl;
  std::cout << "size x: " << colored_scarlett_image.getSize().x << std::endl
            << "size y: " << colored_scarlett_image.getSize().y << std::endl;
}