#include <mutex>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "rov_interfaces/msg/thruster_setpoints.hpp"
#include "rov_interfaces/msg/pwm.hpp"
#include "rov_interfaces/srv/create_continuous_servo.hpp"

#include "eigen3/Eigen/Dense"

using std::placeholders::_1;

using namespace std::chrono_literals;

#define NUM_THRUSTERS (8)

class FlightController : public rclcpp::Node {
public:
    FlightController() : Node(std::string("flight_controller")) {
        _subscription = this->create_subscription<rov_interfaces::msg::ThrusterSetpoints>(
            "thruster_setpoints", 10, std::bind(&FlightController::thruster_setpoint_callback, this, _1));
        _publisher = this->create_publisher<rov_interfaces::msg::PWM>("PWM", 10);

        // use PWM service to register thrusters on PCA9685
        this->registerThrusters();

        // start control loops
        // TODO: figure out how to not use wall_timer
        // _translation = rclcpp::create_timer(*this, std::make_shared<rclcpp::Clock>(), rclcpp::Duration(0.05), std::bind(&FlightController::Translation, this));
        // _translation = this->create_wall_timer(500ms, std::bind(&FlightController::Translation, this));
        // _attitude = rclcpp::create_timer(*this, std::make_shared<rclcpp::Clock>(), rclcpp::Duration(0.05), std::bind(&FlightController::Attitude, this));
        // _attitude = this->create_wall_timer(500ms, std::bind(&FlightController::Attitude, this));
    }
private:
    void registerThrusters() {
        // create service client
        auto pca9685 = this->create_client<rov_interfaces::srv::CreateContinuousServo>("create_continuous_servo");
        for(int i=0; i < NUM_THRUSTERS; i++) {
            // create continuous servo creation requests on channels 0 -> NUM_THRUSTERS
            auto req = std::make_shared<rov_interfaces::srv::CreateContinuousServo_Request>();
            req->channel = i;
            // ensure service is not busy
            while(!pca9685->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    exit(0);
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            }

            // create lambda function to handle asynchronous callbacks
            using ServiceResponseFuture = rclcpp::Client<rov_interfaces::srv::CreateContinuousServo>::SharedFuture;
            auto registerThrusterCallback = [this](ServiceResponseFuture future) {
                auto res = future.get();
                if(res->result)
                    RCLCPP_INFO(this->get_logger(), "Successfully registered continuous servo on channel %i", res->channel);
                else
                    RCLCPP_ERROR(this->get_logger(), "Unsuccessfully registered continuous servo on channel %i", res->channel);
            };
            // asynchronously send these servo creation requests
            auto future_result = pca9685->async_send_request(req, registerThrusterCallback);
        }
    }

    void thruster_setpoint_callback(const rov_interfaces::msg::ThrusterSetpoints::SharedPtr msg) {
        // no race condition!!
        std::lock_guard<std::mutex>(this->setpoint_mutex);
        // fill the matrixes
        translation_setpoints(1,1) = msg->fx;
        translation_setpoints(2,1) = msg->fy;
        translation_setpoints(3,1) = msg->fz;
        attitude_setpoints(1,1) = msg->tx;
        attitude_setpoints(2,1) = msg->ty;
        attitude_setpoints(3,1) = msg->tz;
    }

    void Translation() {
        // TODO: implement linear PID controller that approaches setpoint
    }

    void Attitude() {
        // TODO: implement non linear PI controller that approaches attitude setpoint
    }

    rclcpp::Publisher<rov_interfaces::msg::PWM>::SharedPtr _publisher;
    rclcpp::Subscription<rov_interfaces::msg::ThrusterSetpoints>::SharedPtr _subscription;

    rclcpp::TimerBase::SharedPtr _translation;
    rclcpp::TimerBase::SharedPtr _attitude;

    Eigen::MatrixXd translation_setpoints = Eigen::MatrixXd(3,1);
    Eigen::MatrixXd attitude_setpoints = Eigen::MatrixXd(3,1);
    Eigen::MatrixXd thruster_geometry = Eigen::MatrixXd(6, NUM_THRUSTERS);
    std::mutex setpoint_mutex;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlightController>());
    rclcpp::shutdown();
    return 0;
}
