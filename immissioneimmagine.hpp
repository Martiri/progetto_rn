#ifndef IMMISSIONEIMMAGINE_HPP
#define IMMISSIONEIMMAGINE_HPP
#include <iostream>
#include <vector>
#include <SFML/Graphics.hpp>
#include <filesystem>

 for (const auto& entry : std::filesystem::directory_iterator(immagini)) {
        if (entry.is_regular_file()) {
            sf::Image image;
            if (image.loadFromFile(entry.path().string())) {
                image.push_back(std::move(image));
            }
        }
        return image;
    }







#endif