#include <rclcpp/rclcpp.hpp>

#include "flight_controller/flight_controller.hpp"

#include <SFML/Graphics/Drawable.hpp>
#include <SFML/Graphics/Transformable.hpp>
#include <SFML/Graphics/Sprite.hpp>
#include <SFML/Graphics/Text.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/Window/Event.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <yaml-cpp/yaml.h>

#include <boost/algorithm/string.hpp>
#include <boost/range/adaptors.hpp>

#include <filesystem>
#include <fstream>

struct ThrusterDisplay : public sf::Drawable {
    ThrusterDisplay() {
        std::filesystem::path prefix(ament_index_cpp::get_package_share_directory("flight_controller"));
        
        thrust_vec_tex.loadFromFile((prefix / "assets/" / "thrust_vec_tex.png").string());
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

class FlightDisplay : public rclcpp::Node {
public:
    FlightDisplay() : rclcpp::Node("flight_display") {
        declare_params();

        std::stringstream ss;
        for(int i=0; i<8; i++) {
            ss.str("");
            ss << i;
            thruster_displays[i] = ThrusterDisplay();

            auto to_string = static_cast<std::string(*)(double)>(std::to_string);
            thruster_displays[i].pin.setString(std::to_string(get_parameter("Thruster"+ss.str()+".Pin").as_int()));
            thruster_displays[i].pos.setString(boost::algorithm::join(get_parameter("Thruster"+ss.str()+".Position").as_double_array() | boost::adaptors::transformed(to_string), ", "));
            thruster_displays[i].thrust.setString(boost::algorithm::join(get_parameter("Thruster"+ss.str()+".Thrust").as_double_array() | boost::adaptors::transformed(to_string), ", "));
            RCLCPP_INFO(get_logger(), "Thruster%s\nPin: %s\nPos: %s\nThrust: %s", ss.str().c_str(), thruster_displays[i].pin.getString().toAnsiString().c_str(), thruster_displays[i].pos.getString().toAnsiString().c_str(), thruster_displays[i].thrust.getString().toAnsiString().c_str());
        }
    }
private:
    void declare_params() {
        std::stringstream ss;
        for(int i=0; i<8; i++) {
            ss.str("");
            ss << i;
            declare_parameter("Thruster" + ss.str() + ".Position", std::vector<double>{0,0,0});
            declare_parameter("Thruster" + ss.str() + ".Thrust", std::vector<double>{0,0,0});
            declare_parameter("Thruster" + ss.str() + ".Pin", NULL);
        }
    }

    std::array<ThrusterDisplay, 8> thruster_displays;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    FlightDisplay::SharedPtr display = std::make_shared<FlightDisplay>();

    // FlightController::SharedPtr controller = std::make_shared<FlightController>();

    rclcpp::executors::MultiThreadedExecutor exec;
    // exec.add_node(controller);
    exec.add_node(display);

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
        exec.spin_all(std::chrono::milliseconds(1000/60));

        // clear window
        window.clear();

        // draw thrusters

    }

    rclcpp::shutdown();

    return 0;
}