#include "flight_controller/controller.hpp"
#include <boost/lexical_cast.hpp>

namespace {

void declare_parameters(FlightController* node) {
    // declare thruster parameters
    std::stringstream ss;
    std::string s;
    for(int i = 0; i < NUM_THRUSTERS; i++) {
        ss.str("");
        ss << "Thruster";
        ss << i;
        // default position and thrust vectors to 0
        node->declare_parameters(ss.str(), std::map<std::string, std::vector<double>>{
            {"Position", std::vector<double>{0,0,0}},
            {"Forward", std::vector<double>{0,0,0}}
        });
        // default PWM pin to i in [0,NUM_THRUSTERS]
        node->declare_parameters(ss.str(), std::map<std::string, int>{
            {"PWM_Pin", i}
        });
    }

    node->declare_parameter("Use_PCA9685", true);

    // cascaded pid parameters
    node->declare_parameter("pos_kp", 1.0f);
    node->declare_parameter("pos_ki", 1.0f);
    node->declare_parameter("pos_kd", 1.0f);
    node->declare_parameter("vel_kp", 1.0f);
    node->declare_parameter("vel_ki", 1.0f);
    node->declare_parameter("vel_kd", 1.0f);
}

}

bool wait_for_service_or_timeout(FlightController* controller, rclcpp::Client<rov_interfaces::srv::CreateContinuousServo>::SharedPtr client, std::chrono::seconds timeout) {
    auto time_start = std::chrono::high_resolution_clock::now();
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            return false;
        }

        auto time_current = std::chrono::high_resolution_clock::now();
        RCLCPP_INFO(controller->get_logger(), "Failed to lock pca9685 service, waiting again...");

        if (std::chrono::duration_cast<std::chrono::seconds>(time_current - time_start) > timeout) {
            return false;
        }
    }

    return true;
}

FlightController::FlightController() : Node("flight_controller") {
    // create ros objects
    ::declare_parameters(this);

    pid_update_timer = this->create_wall_timer(std::chrono::milliseconds(16), std::bind(&FlightController::updatePID, this));
    
    // pos PI controller 
    position_pid = PID<Eigen::Vector<float, 6>>(true, true, false);
    // vel PD controller
    velocity_pid = PID<Eigen::Vector<float, 6>>(true, false, true);

    // Construct the thruster geometry matrix
    struct Thruster {
        Eigen::Vector3f pos;
        Eigen::Vector3f fwd;
        uint64_t pin;
    };
    std::string parameter_name_base;
    for (uint64_t i; i < NUM_THRUSTERS; i++) {
        // get thruster parameters and construct helper data type
        parameter_name_base = "Thruster" + boost::lexical_cast<std::string>(i) + ".";
        auto pos = this->get_parameter(parameter_name_base + "Position").as_double_array();
        auto fwd = this->get_parameter(parameter_name_base + "Forward").as_double_array();
        RCLCPP_WARN(this->get_logger(), "%f, %f, %f", pos[0], pos[1], pos[2]);
        RCLCPP_WARN(this->get_logger(), "%f, %f, %f", fwd[0], fwd[1], fwd[2]);
        Thruster thruster = {
            Eigen::Vector3f(static_cast<float>(pos[0]), static_cast<float>(pos[1]), static_cast<float>(pos[2])), 
            Eigen::Vector3f(static_cast<float>(fwd[0]), static_cast<float>(fwd[1]), static_cast<float>(fwd[2])),
            static_cast<uint64_t>(this->get_parameter(parameter_name_base + "PWM_Pin").as_int())
        };
        
        // apply thruster data to internal data structures
        pwm_pin_lookup[i] = thruster.pin;
        Eigen::Vector<float, 6> g_col;
        g_col << thruster.fwd.normalized(), thruster.pos.cross(thruster.fwd.normalized());
        G.col(i) = g_col;
    }

    // calculate condition number to validate thruster geometry
    Eigen::JacobiSVD<Eigen::Matrix<float, 6, NUM_THRUSTERS>> G_svd(G);
    double condition_number = G_svd.singularValues()(0)
        / G_svd.singularValues()(G_svd.singularValues().size()-1);
    if (condition_number > 1e4) {
        RCLCPP_WARN(this->get_logger(), "Poor conditioning of Thruster Geometry Matrix detected");
        RCLCPP_WARN(this->get_logger(), std::string("Condition number: " + boost::lexical_cast<std::string>(condition_number)).c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Well conditioned Thruster Geometry Matrix detected");
        RCLCPP_INFO(this->get_logger(), std::string("Condition number: " + boost::lexical_cast<std::string>(condition_number)).c_str());
    }

    // get qr decomposition of G for least squares thruster control allocation
    G_qr_decomp = G.fullPivHouseholderQr();

    // set default thrust to pwm function
    thrust_to_pwm_function = [](float thrust){return thrust;};

    // create async requests to pca9685 to initalize pwm outputs
    this->pca9685_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    registerThrusters();

    RCLCPP_INFO(this->get_logger(), "Initialization Complete");
}

void FlightController::registerThrusters() {
    auto client = this->create_client<rov_interfaces::srv::CreateContinuousServo>("create_continuous_servo", rmw_qos_profile_services_default, this->pca9685_callback_group);
    static auto pca9685_registration_callback = [this](rclcpp::Client<rov_interfaces::srv::CreateContinuousServo>::SharedFuture future) {
        this->pwm_pin_ready[future.get()->channel] = future.get()->result;
        return;
    };

    for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
        pwm_pin_ready[pwm_pin_lookup[i]] = false;
        // wait for service to be ready, complain and move on if it takes too long
        // TODO: when moving on, spin up some threads dedicated to eventually getting the pins registered so we dont prevent this node from ever functioning
        auto timeout = std::chrono::seconds(3);
        if (!wait_for_service_or_timeout(this, client, timeout)) {
            RCLCPP_WARN(this->get_logger(), std::string("More than " + boost::lexical_cast<std::string>(std::chrono::duration_cast<std::chrono::seconds>(timeout).count()) + " seconds have elapsed when attempting to register thruster,"
            " is the pca9685 node online? Skipping registration of pin " + boost::lexical_cast<std::string>(pwm_pin_lookup[i])).c_str());
            if (rclcpp::ok()) {
                continue;
            }
            return;
        }

        RCLCPP_INFO(this->get_logger(), "bruh");
        auto request = std::make_shared<rov_interfaces::srv::CreateContinuousServo::Request>();
        request->channel = pwm_pin_lookup[i];
        auto future_and_request_id = client->async_send_request(request, pca9685_registration_callback);
        pca9685_response_futures[i] = future_and_request_id.future;
        pca9685_request_ids[i] = future_and_request_id.request_id;
    }
}

void FlightController::updatePID() {
    const float dt = 0.016; // 16 milliseconds per update, TODO: replace with dynamic measurement?
    
    // TODO: implement
    pos_setpoint = Eigen::Vector<float, 6>::Zero();
    auto pos_measurement = Eigen::Vector<float, 6>::Zero();
    auto vel_measurement = Eigen::Vector<float, 6>::Zero();

    // update cascading PID
    Eigen::Vector<float, 6> v_des = position_pid.update(dt, pos_measurement, pos_setpoint);
    F = velocity_pid.update(dt, vel_measurement, v_des);

    // Utilize the QR decomposition of G to solve GT = F, more numerically stable than pseudoinverse
    // TODO: add QP here to refine T?
    // minT ​∥GT−Fcmd​∥^2 subject to Tmin ​≤ Ti ​≤ Tmax​
    T = G_qr_decomp.solve(F);

    // TODO: implement default thrust to pwm functions, currently just returns the input
    for (uint8_t i = 0; i < 6; i++) {
        if (!pwm_pin_ready[pwm_pin_lookup[i]]) continue;
        U(i) = thrust_to_pwm_function(T(i));
        // apply U to PCA9685
        auto msg = std::make_unique<rov_interfaces::msg::PWM>();
        msg->channel = pwm_pin_lookup[i];
        msg->angle_or_throttle = U(i);
        pwm_publisher->publish(std::move(msg));
    }
}

void FlightController::updateSetpoints(rov_interfaces::msg::ThrusterSetpoints::ConstSharedPtr msg) {
    // TODO: change message definition to send desired position and orientation
    pos_setpoint = Eigen::Vector<float, 6>(
        static_cast<float>(0.0f),
        static_cast<float>(0.0f),
        static_cast<float>(0.0f),
        static_cast<float>(0.0f),
        static_cast<float>(0.0f),
        static_cast<float>(0.0f)
    );
}