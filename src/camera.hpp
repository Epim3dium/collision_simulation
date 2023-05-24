#ifndef H_EPI_CAMERA
#define H_EPI_CAMERA
#include "SFML/Graphics/RenderTarget.hpp"
#include "transform.hpp"
#include <SFML/Graphics/View.hpp>
namespace epi {

class Camera : Signal::Observer<TransformEvent> {
    sf::View _view;
    vec2f _size;
    sf::RenderTarget& _window;
public:
    Transform transform;
    virtual void onNotify(TransformEvent event) {
        if(event.isPosChanged)
            _view.setCenter(transform.getPos());
        if(event.isRotChanged)
            _view.setRotation(transform.getRot() / EPI_PI * 180.f);
        if(event.isScaleChanged)
            _view.setSize(_size * transform.getScale());
        _window.setView(_view);
    }
    void Shake() {}
    //smoothing

    Camera(sf::RenderTarget& window, vec2f size) : _size(size), _window(window) {
        _view = window.getDefaultView();
        _view.setSize(_size * transform.getScale());
        window.setView(_view);

        transform.addObserver(this);
    }
};
}

#endif
