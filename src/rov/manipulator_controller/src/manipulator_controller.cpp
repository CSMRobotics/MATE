#include "manipulator_controller/manipulator_controller.hpp"
#include <algorithm>

namespace {
    void declare_manipulator_controller_parameters(rclcpp::Node*const node) {
        node->declare_parameter("wrist_invert", false);
        node->declare_parameter("wrist_lower_bound", 0.0);
        node->declare_parameter("wrist_upper_bound", 180.0);
        node->declare_parameter("wrist_pwm_pin", 14);
        node->declare_parameter("clamp_invert", false);
        node->declare_parameter("clamp_lower_bound", 0.0);
        node->declare_parameter("clamp_upper_bound", 180.0);
        node->declare_parameter("clamp_pwm_pin", 11);
    }

    void clamp(float* toClamp, double lower, double upper) {
        *toClamp = std::min(*toClamp, static_cast<float>(upper));
        *toClamp = std::max(*toClamp, static_cast<float>(lower));
    }

    auto registration_callback = [](rclcpp::Client<rov_interfaces::srv::CreateServo>::SharedFuture future) {
        auto result = future.get();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully registered servo on channel %i", result->channel);
    };
}

ManipulatorController::ManipulatorController() : Node(std::string("manipulator_controller")) {
    declare_manipulator_controller_parameters(this);

    this->pca9685_registration_callbackgroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->pca9685_client = this->create_client<rov_interfaces::srv::CreateServo>("create_servo", rmw_qos_profile_services_default, this->pca9685_registration_callbackgroup);
    this->register_servos();

    manip_setpoints = this->create_subscription<rov_interfaces::msg::ManipulatorSetpoints>("manipulator_setpoints", 10, std::bind(&ManipulatorController::setpoint_callback, this, std::placeholders::_1));
    pwm_pub = this->create_publisher<rov_interfaces::msg::PWM>("PWM", 10);
}

void ManipulatorController::register_servos() {
    // create servo creation requests on channels in thruster pwm pin map
    auto servo1 = std::make_shared<rov_interfaces::srv::CreateServo_Request>();
    auto servo2 = std::make_shared<rov_interfaces::srv::CreateServo_Request>();

    servo1->channel = this->get_parameter("wrist_pwm_pin").as_int();
    servo2->channel = this->get_parameter("clamp_pwm_pin").as_int();

    // ensure service is not busy
    while(!pca9685_client->wait_for_service(std::chrono::milliseconds(100))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            exit(0);
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    // asynchronously send these servo creation requests
    pca9685_requests[0] = pca9685_client->async_send_request(servo1, registration_callback);
    pca9685_requests[1] = pca9685_client->async_send_request(servo2, registration_callback);
}

void ManipulatorController::setpoint_callback(const rov_interfaces::msg::ManipulatorSetpoints::SharedPtr msg) {
    static float wrist_pos_last = 90;
    static float clamp_pos_last = 90;
    // map values from rov_control
    float wrist = wrist_pos_last += msg->wrist * ((this->get_parameter("wrist_invert").as_bool() << 1) - 1);
    float clamp = clamp_pos_last += msg->clamp * ((this->get_parameter("clamp_invert").as_bool() << 1) - 1);

    // clamp the received values
    ::clamp(&wrist, this->get_parameter("wrist_lower_bound").as_double(), this->get_parameter("wrist_upper_bound").as_double());
    ::clamp(&clamp, this->get_parameter("clamp_lower_bound").as_double(), this->get_parameter("clamp_upper_bound").as_double());

    // publish PWM
    // wrist pwm
    auto pwm = rov_interfaces::msg::PWM();
    pwm.angle_or_throttle = wrist;
    pwm.channel = this->get_parameter("wrist_pwm_pin").as_int();
    pwm_pub->publish(pwm);

    // clamp pwm
    pwm = rov_interfaces::msg::PWM();
    pwm.angle_or_throttle = clamp;
    pwm.channel = this->get_parameter("clamp_pwm_pin").as_int();
    pwm_pub->publish(pwm);
}