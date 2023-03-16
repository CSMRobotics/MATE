#include <rclcpp/rclcpp.hpp>

#include "flight_controller/flight_controller.hpp"

#include <SFML/Graphics/Drawable.hpp>
#include <SFML/Graphics/Transformable.hpp>
#include <SFML/Graphics/Sprite.hpp>
#include <SFML/Graphics/Text.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/Window/Event.hpp>

struct ThrusterDisplay : public sf::Drawable {
    ThrusterDisplay() {
        thrust_vec_tex.loadFromFile("thrust_vec_tex.png");
        thrust_vec.setTexture(thrust_vec_tex, true);
    }

    sf::Text pos;
    sf::Text thrust;
    sf::Text pin;
    sf::Text throttle;
    sf::Text pwm;
    sf::Sprite thrust_vec;
    sf::Texture thrust_vec_tex;
protected:
    void draw(sf::RenderTarget& target, sf::RenderStates states) const {
        target.draw(thrust_vec, states);
    };
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // FlightController::SharedPtr controller = std::make_shared<FlightController>();

    // rclcpp::executors::MultiThreadedExecutor exec;
    // exec.add_node(controller);

    sf::RenderWindow window(sf::VideoMode(800,600), "Flight Display");

    while(window.isOpen()) {
        // process window events
        sf::Event ev;
        while(window.pollEvent(ev)) {
            switch(ev.type) {
                case sf::Event::Closed:
                    window.close();
                    break;
                case sf::Event::KeyPressed:
                    if(ev.key.code == sf::Keyboard::Q || ev.key.code == sf::Keyboard::Escape) {
                        window.close();
                    }
                    break;
                default:
                    break;
            }
        }

        // update nodes
        // exec.spin_some(std::chrono::milliseconds(1000/60)); 

        // clear window
        window.clear();

        // draw thrusters

    }

    rclcpp::shutdown();

    return 0;
}