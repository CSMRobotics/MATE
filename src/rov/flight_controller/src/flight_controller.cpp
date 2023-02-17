#include "flight_controller/flight_controller.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <sstream>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "rov_interfaces/srv/create_continuous_servo.hpp"

namespace {
void declare_params(rclcpp::Node* node) {
    std::stringstream ss;
    std::string s;
    for(int i = 0; i < 8; i++) {
        ss.clear();
        ss << "Thruster";
        ss << i;
        node->declare_parameter(ss.str());
    }
}
}

FlightController::FlightController() : Node(std::string("flight_controller")) {
    // initialize parameters for this Node
    declare_params(this);

    // using ROS parameters, grab the thruster data provided and construct geometry matrix, coefficient matrix, and PWM pin map
    std::stringstream ss;
    std::array<Eigen::VectorXd, NUM_THRUSTERS> temp;
    for(int i = 0; i < NUM_THRUSTERS; i++) {
        ss.clear();
        ss << "Thruster" << i << "\\";
        // Construct internal thruster data type from ROS parameter
        Thruster t = {
            Eigen::Vector3d(this->get_parameter(ss.str()+"Position").as_double_array().data()),
            Eigen::Vector3d(this->get_parameter(ss.str()+"Thrust").as_double_array().data()),
            this->get_parameter(ss.str()+"Pin").as_int()
        };
        thrusters[i] = t;
        // TODO: look at this again
        // calculate linear and rotation contribution
        Eigen::Vector3d linear_contribution(t.thrust * MAX_THRUST_VALUE);
        Eigen::Vector3d rotation_contribution(t.position.cross(t.thrust * MAX_THRUST_VALUE));

        // add the PWM pin to the thruster index map
        this->thruster_index_to_PWM_pin.emplace(std::make_pair(i, t.pwm_pin));

        // add the thruster's geometry to the thruster geometry matrix
        temp[i] = Eigen::VectorXd(6);
        temp[i] << linear_contribution, rotation_contribution;
        thruster_geometry.col(i) << temp[i];
    }
    
    // compute the pseudo inverse of the thruster geometry
    this->thruster_geometry_pseudo_inverse = this->thruster_geometry.transpose() * (this->thruster_geometry*this->thruster_geometry.transpose()).inverse();
    this->thruster_coefficient_matrix.diagonal() << 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0; // thrusters are identical T200s running at ~12V
    this->thruster_coefficient_matrix_times_geometry = this->thruster_coefficient_matrix * this->thruster_geometry_pseudo_inverse;

    // use PWM service to register thrusters on PCA9685
    this->registerThrusters();
    
    // create ros subscriptions and publishers
    // receives translation and rotational setpoints from rov_control
    thruster_setpoint_subscription = this->create_subscription<rov_interfaces::msg::ThrusterSetpoints>("thruster_setpoints", 10, std::bind(&FlightController::setpoint_callback, this, std::placeholders::_1));
    // receives orientation and acceleration data from bno055
    bno_data_subscription = this->create_subscription<rov_interfaces::msg::BNO055Data>("bno055_data", rclcpp::SensorDataQoS(), std::bind(&FlightController::bno_callback, this, std::placeholders::_1));
    // publishes PWM commands to PCA9685
    _publisher = this->create_publisher<rov_interfaces::msg::PWM>("pwm", 10);

    // about 60 hz update rate
    // TODO: Check that service changes the timer callback
    pid_control_loop = this->create_wall_timer(std::chrono::milliseconds(16), FlightController::_update);

    // Creates service responsible for toggling between updateSimple and updatePID
    toggle_PID_service = this->create_service<std_srvs::srv::Empty>("toggle_pid", 
        std::bind(&FlightController::toggle_PID, this, std::placeholders::_1, std::placeholders::_2));
}

void FlightController::registerThrusters() {
    // create service client
    auto pca9685 = this->create_client<rov_interfaces::srv::CreateContinuousServo>("create_continuous_servo");
    std::array<std::shared_future<std::shared_ptr<rov_interfaces::srv::CreateContinuousServo_Response>>, NUM_THRUSTERS> requests;
    for(int i=0; i < NUM_THRUSTERS; i++) {
        // create continuous servo creation requests on channels in thruster pwm pin map
        auto req = std::make_shared<rov_interfaces::srv::CreateContinuousServo_Request>();
        try {
            req->channel = thruster_index_to_PWM_pin.at(i);
        } catch (std::out_of_range const& e) {
            // if thruster index not in map, throw error
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Thruster index's PWM pin %i not found in thruster PWM pin map", i);
        }
        // ensure service is not busy
        while(!pca9685->wait_for_service(std::chrono::milliseconds(100))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                exit(0);
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        // asynchronously send these servo creation requests
        requests[i] = pca9685->async_send_request(req);
    }

    // wait for requests to be completed
    for(int i=0; i < NUM_THRUSTERS; i++) {
        auto status = requests[i].wait_for(std::chrono::milliseconds(100));
        if(status == std::future_status::ready) {
            auto res = requests[i].get();
            if(res->result) {
                RCLCPP_INFO(this->get_logger(), "Successfully registered continuous servo on channel %i", res->channel);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Could not register continuous servo on channel %i", res->channel);
            }
        } else {
            i--;
            continue;
        }
    }
}

void FlightController::toggle_PID(const std_srvs::srv::Empty_Request::SharedPtr request, 
        std_srvs::srv::Empty_Response::SharedPtr response) {
    typedef void (fntype)(void);
    fntype** ptr = _update.target<fntype*>();
    if(ptr == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "Flight Controller update target is nullptr. Defaulting to Simple");
        _update = std::bind(&FlightController::updateSimple, this);
        return;
    } else {
        std::swap(_update, _update2);
    }
}

void FlightController::setpoint_callback(const rov_interfaces::msg::ThrusterSetpoints::SharedPtr setpoints) {
    std::lock_guard<std::mutex>(this->setpoint_mutex);
    std::lock_guard<std::mutex>(this->stall_mutex);
    translation_setpoints(0,0) = setpoints->vx; // [-1,1]
    translation_setpoints(1,0) = setpoints->vy; // ^
    translation_setpoints(2,0) = setpoints->vz; // ^
    attitude_setpoints(0,0) = setpoints->omegax; // [-1,1]
    attitude_setpoints(1,0) = setpoints->omegay; // ^
    attitude_setpoints(2,0) = setpoints->omegaz; // ^
}

void FlightController::bno_callback(const rov_interfaces::msg::BNO055Data::SharedPtr bno_data) {
    std::lock_guard<std::mutex>(this->bno_mutex);
    this->bno_data = *bno_data.get();
}

void FlightController::updateSimple() {
    // convert setpoints to actuations that the controller must apply
    Eigen::Matrix<double, NUM_THRUSTERS, 1> unnormalized_actuations = Eigen::Matrix<double, NUM_THRUSTERS, 1>::Zero();
    double largest_abs_actuation_request = 0;
    for(int i = 0; i < NUM_THRUSTERS; i++) {
        Thruster t = thrusters[i];
        // TODO: VERIFY THIS WORKS AS INTENDED => max value SHOULD be +-1 (rov_control.py only updates translation or rotation)
        // Actuation = Linear Actuation(0) + Rotational Actuation (0)
        // ALTERNATIVELY: G^+ * tau = activation for rotation or translation defined but not both
        unnormalized_actuations(i,0) = t.thrust.dot(translation_setpoints.normalized()) 
            + t.position.cross(t.thrust).dot(attitude_setpoints.normalized());

        // find the abs largest actuation request
        // used after to normalize the actuations ensuring that |actuation| <= 1 holds for all actuations
        if(abs(unnormalized_actuations(i,0)) > largest_abs_actuation_request) {
            largest_abs_actuation_request = abs(unnormalized_actuations(i,0));
        }
    }
    
    // scale actuations by abs of largest actuation request
    Eigen::Matrix<double, NUM_THRUSTERS, 1> actuations = unnormalized_actuations / largest_abs_actuation_request;

    // apply the actuations to the thrusters
    // publish PWM values
    for(int i = 0; i < NUM_THRUSTERS; i++) {
        rov_interfaces::msg::PWM msg;
        msg.angle_or_throttle = static_cast<float>(actuations(i,0)); // yeah we convert from a double to a float :(
        msg.channel = thruster_index_to_PWM_pin.at(i);
        _publisher->publish(msg);
#ifdef DEBUG
        RCLCPP_INFO(this->get_logger(), "PIN:%i THROTTLE:%d", thruster_index_to_PWM_pin.at(i), actuations(i,0));
#endif
    }
}

void FlightController::updatePID() {
    Eigen::Vector3d desired_force;
    Eigen::Vector3d desired_torque;

    // update dt
    auto now = std::chrono::high_resolution_clock::now();
    int dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - this->last_updated).count();
    this->last_updated = now;

    // fill the matrixes based on bno and setpoint data
    this->setpoint_mutex.lock();
    this->bno_mutex.lock();
    Eigen::Vector3d linear_accel(bno_data.linearaccel.i, bno_data.linearaccel.j,bno_data.linearaccel.k); // m/s^2
    Eigen::Vector3d linear_velocity = linear_velocity + linear_accel * dt_ms / 1000; // get an approximation of linear velocity
    linear_velocity_err_last = linear_velocity_err;
    linear_velocity_err[0] = translation_setpoints(0,0) - linear_velocity[0];
    linear_velocity_err[1] = translation_setpoints(1,0) - linear_velocity[1];
    linear_velocity_err[2] = translation_setpoints(2,0) - linear_velocity[2];
    Eigen::Vector3d ha = dt_ms * 0.5 * attitude_setpoints; // vector of half angle
    Eigen::Vector3d omega = Eigen::Vector3d(bno_data.gyroscope.i, bno_data.gyroscope.j, bno_data.gyroscope.k);
    auto quaternion_measured = Eigen::Quaterniond(bno_data.orientation.w, bno_data.orientation.i, bno_data.orientation.j, bno_data.orientation.k);
    this->setpoint_mutex.unlock();
    this->bno_mutex.unlock();

    // update desired force
    // Linear, discrete-time PID controller
    // lambdas for linear PID loop
    auto P = [](double& kp, Eigen::Vector3d& linear_velocity_err)->Eigen::Vector3d {
        return kp * linear_velocity_err;
    };
    auto I = [](double& ki, Eigen::Vector3d& linear_velocity_err, int& dt_ms)->Eigen::Vector3d {
        return ki * linear_velocity_err * dt_ms / 1000;
    };
    auto D = [](double& kd, Eigen::Vector3d& linear_velocity_err, Eigen::Vector3d& linear_velocity_err_last, int& dt_ms)->Eigen::Vector3d {
        return kd* (linear_velocity_err - linear_velocity_err_last) / (static_cast<double>(dt_ms) / 1000);
    };

    // P + I + D
    desired_force = P(kp,linear_velocity_err)
                    + (linear_integral += I(ki, linear_velocity_err, dt_ms))
                    + D(kd, linear_velocity_err, linear_velocity_err_last, dt_ms);

    // update desired torque
    // TODO: look at this if performance needs to be improved (loses accuracy tho)
    // see https://stackoverflow.com/questions/24197182/efficient-quaternion-angular-velocity/24201879#24201879
    double l = ha.norm(); // magnitude
    if (l > 0) {
        ha *= sin(l) / l;
        quaternion_reference = Eigen::Quaterniond(cos(l), ha.x(), ha.y(), ha.z());
    } else {
        quaternion_reference = Eigen::Quaterniond(1.0, ha.x(), ha.y(), ha.z());
    }
    // desired orientation is only known by the pilot, quaternion reference defines a reference from a unity quaternion (1,0,0,0)
    // to get actual reference, this must be multiplied with the measured quaternion to get desired orientation
    // TODO: ensure that when w = 0, this is close to, or equal to unity
    quaternion_reference = quaternion_reference * quaternion_measured;

    // Non Linear P^2 Quaternion based control scheme
    // For derivation of controller see: http://www.diva-portal.org/smash/get/diva2:1010947/FULLTEXT01.pdf
    // For derivation of dynamics and control allocation see: https://flex.flinders.edu.au/file/27aa0064-9de2-441c-8a17-655405d5fc2e/1/ThesisWu2018.pdf
    auto q_err = quaternion_reference * quaternion_measured.conjugate(); // hamilton product (hopefully)
    Eigen::Vector3d axis_err;

    if(q_err.w() < 0) { // this could be simplified to a negation but eh
        axis_err = q_err.conjugate().vec();
    } else {
        axis_err = q_err.vec();
    }

    desired_torque = (-Pq * axis_err) - (Pw * omega);

    // control allocation
    // u = K^-1 * T^+ * t
    // 8x1 = 8x8 * 8x6 * 6x1 :)
    Eigen::Matrix<double, 6, 1> forcesAndTorques;
    forcesAndTorques << desired_force, desired_torque;

    //TODO:fix
    // solve Ax = b and normalize thrust such that it satisfies MIN_THRUST_VALUE <= throttles[j] <= MAX_THRUST_VALUE 
    // while scaling thrusters to account for large thrust demands on a single thruster
    // Eigen::Matrix<double, NUM_THRUSTERS, 1> throttles = thrust2throttle((this->thruster_coefficient_matrix_times_geometry) * forcesAndTorques);

    // // publish PWM values
    // for(int i = 0; i < NUM_THRUSTERS; i++) {
    //     rov_interfaces::msg::PWM msg;
    //     msg.angle_or_throttle = static_cast<float>(throttles(i,0)); // this is a source of noise in output signals, may cause system instability??
    //     msg.channel = thruster_index_to_PWM_pin.at(i);
    //     _publisher->publish(msg);
// #ifdef DEBUG
//             RCLCPP_INFO(this->get_logger(), "PIN:%i THROTTLE:%d", thruster_index_to_PWM_pin.at(i), actuations(i,0));
// #endif
    // }
}

void FlightController::normalizethrottles(Eigen::Matrix<double,NUM_THRUSTERS,1>* throttles) {
    Eigen::Index loc;
    throttles->minCoeff(&loc);
    double ratioA = std::abs(-1.0 / std::min((*throttles)(loc), -1.0));
    throttles->maxCoeff(&loc);
    double ratioB = std::abs(1.0 / std::max((*throttles)(loc), 1.0));
    if(ratioA < ratioB) {
        *throttles = *throttles * ratioA;
    } else {
        *throttles = *throttles * ratioB;
    }
}