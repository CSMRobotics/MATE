#include "rclcpp/rclcpp.hpp"
#include "rov_interfaces/msg/setpoints.hpp"
#include "rov_interfaces/msg/pwm.hpp"

using std::placeholders::_1;

class ManipulatorControl : public rclcpp::Node {
public:
    ManipulatorControl() : Node(std::string("manipulator_control")) {
        _subscription = this->create_subscription<rov_interfaces::msg::Setpoints>(
            "manipulator_setpoints", 10, std::bind(&ManipulatorControl::topic_callback, this, _1));
        _publisher = this->create_publisher<rov_interfaces::msg::PWM>("PWM", 10);
    }

private:
    void topic_callback(const rov_interfaces::msg::Setpoints::SharedPtr msg) const {
        // do stuff with setpoints
    }
    rclcpp::Publisher<rov_interfaces::msg::PWM>::SharedPtr _publisher;
    rclcpp::Subscription<rov_interfaces::msg::Setpoints>::SharedPtr _subscription;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManipulatorControl>());
    rclcpp::shutdown();
    return 0;
}