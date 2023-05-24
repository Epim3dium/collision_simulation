#ifndef IO_MANAGER_H
#define IO_MANAGER_H
#include "SFML/Graphics/RenderTarget.hpp"
#include "SFML/Graphics/RenderWindow.hpp"
#include "SFML/Window/Event.hpp"
#include "SFML/Window/Mouse.hpp"
#include "SFML/Window/VideoMode.hpp"
#include "SFML/Window/Window.hpp"
#include "SFML/Window.hpp"

#include "types.hpp"
#include "debug.hpp"

namespace epi {
/*
* \brief class managing window input and output, derived from GameObject
*/
struct IOManager;
struct IOManagerEvent {
    IOManager& manager;
    sf::Event event;
};
class IOManager : public Signal::Subject<IOManagerEvent> {
    sf::Event _current_event;
    sf::RenderWindow _window;
public:

    sf::RenderTarget& getRenderObject() {
        return _window;
    }
    sf::Window& getWindow() {
        return _window;
    }
    const sf::Event& getEvent() const {
        return _current_event;
    }
    vec2f getMouseWorldPos() const {
        return _window.mapPixelToCoords(sf::Mouse::getPosition(_window));
    }
    void display() {
        //ImGui::SFML::Update();
        ImGui::SFML::Render(_window);
        _window.display();
    }
    void pollEvents() {
        while(_window.pollEvent(_current_event)) {
            ImGui::SFML::ProcessEvent(_window, _current_event);
            notify({*this, _current_event});
        }
    }
    IOManager(vec2i size, std::string title = "EpiSim") : _window(sf::VideoMode(size.x, size.y), title) 
    { 
        if(!ImGui::SFML::Init(_window)) {
            Log(LogLevel::ERROR) << "failed to initialize ImGui";
        }

    }
    ~IOManager() {
    }
};
}
#endif
