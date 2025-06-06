#include "rov_flight_controller/flight_controller.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <sstream>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#ifndef DEBUG_OUTPUT
#define DEBUG_OUTPUT false
#endif

namespace {
void declare_params(rclcpp::Node* node) {
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
            {"Thrust", std::vector<double>{0,0,0}}
        });
        // default PWM pin to i in [0,NUM_THRUSTERS]
        node->declare_parameters(ss.str(), std::map<std::string, int>{
            {"Pin", i}
        });
    }

    node->declare_parameter("Debug", false);
    node->declare_parameter("Use_PCA9685", true);
    
    // declare gain constant parameters
    // TODO: tune these constants
    node->declare_parameter("kpd", 1.0f);
    node->declare_parameter("kppq", 1.0f);
    node->declare_parameter("kdpq", 0.0f);
}
}

FlightController::FlightController() : Node(std::string("flight_controller")) {
    // initialize parameters for this Node
    declare_params(this);

    // using ROS parameters, grab the thruster data provided and construct geometry matrix, coefficient matrix, and PWM pin map
    std::stringstream ss;
    std::array<Eigen::VectorXd, NUM_THRUSTERS> temp;
    for(int i = 0; i < NUM_THRUSTERS; i++) {
        ss.str("");
        ss << "Thruster" << i << '.';
        // Construct internal thruster data type from ROS parameter
        Thruster t = {
            Eigen::Vector3d(this->get_parameter(ss.str()+"Position").as_double_array().data()),
            Eigen::Vector3d(this->get_parameter(ss.str()+"Thrust").as_double_array().data()),
            this->get_parameter(ss.str()+"Pin").as_int()
        };
        thrusters[i] = t;
        // TODO: look at this again
        // calculate linear and rotation contribution
        Eigen::Vector3d linear_contribution(t.thrust);
        Eigen::Vector3d rotation_contribution(t.position.cross(t.thrust));

        // add the PWM pin to the thruster index map
        this->thruster_index_to_PWM_pin.emplace(std::make_pair(i, t.pwm_pin));
#if DEBUG_OUTPUT
        RCLCPP_DEBUG(this->get_logger(), "Thruster %i: linear: %f, %f, %f  rotation: %f, %f, %f\n",
            i,
            linear_contribution(0,0),
            linear_contribution(1,0),
            linear_contribution(2,0),
            rotation_contribution(0,0),
            rotation_contribution(1,0),
            rotation_contribution(2,0));
#endif

        // add the thruster's geometry to the thruster geometry matrix
        temp[i] = Eigen::VectorXd(6);
        temp[i] << linear_contribution, rotation_contribution;
        thruster_geometry.col(i) << temp[i];
    }
    
    // compute the pseudo inverse of the thruster geometry
    this->thruster_geometry_pseudo_inverse = this->thruster_geometry.transpose() * (this->thruster_geometry*this->thruster_geometry.transpose()).inverse();
    this->thruster_coefficient_matrix.diagonal() << 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0; // thrusters are identical T200s running at ~12V
    this->thruster_coefficient_matrix_times_geometry = this->thruster_coefficient_matrix.inverse() * this->thruster_geometry_pseudo_inverse;

    // use PWM service to register thrusters on PCA9685
    if(this->get_parameter("Use_PCA9685").as_bool()) {
        this->pca9685_registration_callbackgroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        this->pca9685_client = this->create_client<rov_interfaces::srv::CreateContinuousServo>("create_continuous_servo", rmw_qos_profile_services_default, this->pca9685_registration_callbackgroup);
        this->registerThrusters();
    }
    
    // create ros subscriptions and publishers
    // receives translation and rotational setpoints from rov_control
    thruster_setpoint_subscription = this->create_subscription<rov_interfaces::msg::ThrusterSetpoints>("thruster_setpoints", 10, std::bind(&FlightController::setpoint_callback, this, std::placeholders::_1));
    // receives orientation and acceleration data from bno055
    bno_data_subscription = this->create_subscription<rov_interfaces::msg::BNO055Data>("bno055_data", rclcpp::SensorDataQoS(), std::bind(&FlightController::bno_callback, this, std::placeholders::_1));
    // depth data from bar30
    bar_data_subscription = this->create_subscription<rov_interfaces::msg::Bar30Data>("bar30_data", rclcpp::SensorDataQoS(), std::bind(&FlightController::bar_callback, this, std::placeholders::_1));
    // temperature data from tsys01
    tsys_data_subscription = this->create_subscription<rov_interfaces::msg::TSYS01Data>("tsys01_data", rclcpp::SensorDataQoS(), std::bind(&FlightController::tsys_callback, this, std::placeholders::_1));
    // publishes PWM commands to PCA9685
    pwm_publisher = this->create_publisher<rov_interfaces::msg::PWM>("pwm", 10);

    RCLCPP_INFO(this->get_logger(), "Flight Controller Successfully Initialized");
    // about 60 hz update rate
    // TODO: Check that service changes the timer callback
    this->startUpdateLoopTime = std::chrono::high_resolution_clock::now() + std::chrono::seconds(5);
    control_loop = this->create_wall_timer(std::chrono::milliseconds(UPDATE_MS), std::bind(&FlightController::updateNone, this));

    // Creates service responsible for toggling between updateSimple and updatePID
    toggle_PID_service = this->create_service<std_srvs::srv::Empty>("toggle_pid", 
        std::bind(&FlightController::toggle_PID, this, std::placeholders::_1, std::placeholders::_2));
    depth_tare_service = this->create_service<std_srvs::srv::Empty>("tare_depth",
        std::bind(&FlightController::depthTare, this, std::placeholders::_1, std::placeholders::_2));
    safe_service = this->create_service<std_srvs::srv::Empty>("safe",
        std::bind(&FlightController::depthTare, this, std::placeholders::_1, std::placeholders::_2));
    desafe_service = this->create_service<std_srvs::srv::Empty>("desafe",
        std::bind(&FlightController::depthTare, this, std::placeholders::_1, std::placeholders::_2));
}

FlightController::~FlightController() {
    control_loop = this->create_wall_timer(std::chrono::milliseconds(UPDATE_MS), [](){});
    
    // safe the thrusters by turning them to 0 pwm
    rov_interfaces::msg::PWM msg;
    msg.angle_or_throttle = 0;

    for(int i=0; i < NUM_THRUSTERS; i++) {
        msg.channel = i;
        this->pwm_publisher->publish(msg);
    }
}

void FlightController::registerThrusters() {
    auto registration_callback = [logger = this->get_logger()](rclcpp::Client<rov_interfaces::srv::CreateContinuousServo>::SharedFutureWithRequest future) {
        auto result = future.get();
        RCLCPP_INFO(logger, "Successfully registered continuous servo on channel %i", result.second->channel);
    };

    for(int i=0; i < NUM_THRUSTERS; i++) {
        // create continuous servo creation requests on channels in thruster pwm pin map
        auto req = std::make_shared<rov_interfaces::srv::CreateContinuousServo::Request>();

        try {
            req->channel = thruster_index_to_PWM_pin.at(i);
        } catch (std::out_of_range const& e) {
            // if thruster index not in map, throw error
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Thruster index's PWM pin %i not found in thruster PWM pin map", i);
            continue;
        }
        // ensure service is not busy
        while(!pca9685_client->wait_for_service(std::chrono::milliseconds(100))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                exit(0);
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        // asynchronously send these servo creation requests
        pca9685_requests[i] = std::move(pca9685_client->async_send_request(req, std::move(registration_callback)).future);
    }
}

// TODO: fix the PID controller not liking being paused. (set a pause flag? or reset last_updated to now?), dt continues to grow.
void FlightController::toggle_PID(const std_srvs::srv::Empty_Request::SharedPtr request, 
        std_srvs::srv::Empty_Response::SharedPtr response) {
    (void) request;
    (void) response;
    this->last_updated = std::chrono::high_resolution_clock::now();
    control_loop = this->create_wall_timer(std::chrono::milliseconds(UPDATE_MS), FlightController::_update_stab);
}

// TODO: update to new mapping
void FlightController::setpoint_callback(const rov_interfaces::msg::ThrusterSetpoints::SharedPtr setpoints) {
    const std::lock_guard<std::mutex> l1(this->setpoint_mutex);
    const std::lock_guard<std::mutex> l2(this->stall_mutex);
    translation_setpoints(0,0) = setpoints->vx; // [-1,1]
    translation_setpoints(1,0) = setpoints->vy; // ^
    translation_setpoints(2,0) = setpoints->vz; // ^
    attitude_setpoints(0,0) = setpoints->omegax; // [-1,1]
    attitude_setpoints(1,0) = setpoints->omegay; // ^
    attitude_setpoints(2,0) = setpoints->omegaz; // ^
    keep_depth = setpoints->keep_depth;
    free_orientation = setpoints->free_orientation;
}

void FlightController::bno_callback(const rov_interfaces::msg::BNO055Data::SharedPtr bno_data) {
    const std::lock_guard<std::mutex> l1(this->bno_mutex);
    this->bno_data = bno_data;
}

void FlightController::bar_callback(const rov_interfaces::msg::Bar30Data::SharedPtr bar_data) {
    const std::lock_guard<std::mutex> l1(this->bar30_mutex);
    this->bar30_data = bar_data;
}

void FlightController::tsys_callback(const rov_interfaces::msg::TSYS01Data::SharedPtr tsys_data) {
    const std::lock_guard<std::mutex> l1(this->tsys_mutex);
    this->tsys_data = tsys_data;
}

void FlightController::updateNone() {
    // wait 5 seconds before starting
    if(std::chrono::high_resolution_clock::now() >= this->startUpdateLoopTime) {
        control_loop = this->create_wall_timer(std::chrono::milliseconds(UPDATE_MS), FlightController::_update_simple);
    }

    // safe the thrusters by turning them to 0 pwm
    rov_interfaces::msg::PWM msg;
    msg.angle_or_throttle = 0;

    for(int i=0; i < NUM_THRUSTERS; i++) {
        msg.channel = i;
        this->pwm_publisher->publish(msg);
    }
}
void FlightController::safe(const std_srvs::srv::Empty_Request::SharedPtr request, std_srvs::srv::Empty_Response::SharedPtr response) {
    (void) request;
    (void) response;
    // change the update loop to do nothing
    control_loop = this->create_wall_timer(std::chrono::milliseconds(UPDATE_MS), [](){});

    // safe the thrusters by turning them to 0 pwm
    rov_interfaces::msg::PWM msg;
    msg.angle_or_throttle = 0;

    for(int i=0; i < NUM_THRUSTERS; i++) {
        msg.channel = i;
        this->pwm_publisher->publish(msg);
    }
}

void FlightController::desafe(const std_srvs::srv::Empty_Request::SharedPtr request, std_srvs::srv::Empty_Response::SharedPtr response) {
    (void) request;
    (void) response;
    // restart the update loop to be update_simple
    control_loop = this->create_wall_timer(std::chrono::milliseconds(UPDATE_MS), FlightController::_update_simple);
}

void FlightController::updateSimple() {
    // convert setpoints to actuations that the controller must apply
    Eigen::Matrix<double, NUM_THRUSTERS, 1> unnormalized_actuations = Eigen::Matrix<double, NUM_THRUSTERS, 1>::Zero();
    for(int i = 0; i < NUM_THRUSTERS; i++) {
        Thruster t = thrusters[i];
        // TODO: VERIFY THIS WORKS AS INTENDED => max value SHOULD be +-1 (rov_control.py only updates translation or rotation)
        // Actuation = Linear Actuation(0) + Rotational Actuation (0)
        // ALTERNATIVELY: G^+ * tau = activation for rotation or translation defined but not both
        unnormalized_actuations(i,0) = t.thrust.dot(translation_setpoints)
            + t.position.cross(t.thrust).dot(attitude_setpoints);
    }
    
    // scale actuations by abs of largest actuation request
    Eigen::Matrix<double, NUM_THRUSTERS, 1> actuations = unnormalized_actuations;
    // Eigen::Index idx;
    // (actuations.cwiseAbs()).maxCoeff(&idx);
    // if(actuations(idx) != 0)
    //     actuations /= actuations(idx);
#if DEBUG_OUTPUT
    RCLCPP_DEBUG(this->get_logger(), "ACTUATIONS: %f | %f | %f | %f | %f | %f | %f | %f\n", 
        actuations(0,0),
        actuations(1,0),
        actuations(2,0),
        actuations(3,0),
        actuations(4,0),
        actuations(5,0),
        actuations(6,0),
        actuations(7,0));
#endif
    clampthrottles(&actuations);
#if DEBUG_OUTPUT
    RCLCPP_DEBUG(this->get_logger(), "NORMALIZED: %f | %f | %f | %f | %f | %f | %f | %f\n", 
        actuations(0,0),
        actuations(1,0),
        actuations(2,0),
        actuations(3,0),
        actuations(4,0),
        actuations(5,0),
        actuations(6,0),
        actuations(7,0));
#endif

    // apply the actuations to the thrusters
    // publish PWM values
    // TODO: consider refactoring this to have 8 separate publishers?
    // publishers would be created upon successful return from registerThrusters() if Use_PCA9685 == true
    // else defaults would be created from params.yaml
    // this method would allow for rqt_topic to more accurately collect PWM values
    for(int i = 0; i < NUM_THRUSTERS; i++) {
        rov_interfaces::msg::PWM msg;
        msg.angle_or_throttle = static_cast<float>(actuations(i,0)); // yeah we convert from a double to a float :(
        msg.channel = thruster_index_to_PWM_pin.at(i);
        pwm_publisher->publish(msg);
#if DEBUG_OUTPUT
        RCLCPP_DEBUG(this->get_logger(), "PIN:%i THROTTLE:%f", thruster_index_to_PWM_pin.at(i), actuations(i,0));
#endif
    }
}

// void FlightController::updatePID() {
//     Eigen::Vector3d desired_force;
//     Eigen::Vector3d desired_torque;

//     // update dt
//     auto now = std::chrono::high_resolution_clock::now();
//     int dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - this->last_updated).count();
//     this->last_updated = now;

//     // fill the matrixes based on bno and setpoint data
//     this->setpoint_mutex.lock();
//     this->bno_mutex.lock();
//     Eigen::Vector3d linear_accel(bno_data->imu.linear_acceleration.x, bno_data->imu.linear_acceleration.y,bno_data->imu.linear_acceleration.z); // m/s^2
//     linear_velocity += linear_accel * dt_ms / 1000; // get an approximation of linear velocity
//     Eigen::Vector3d ha = dt_ms * 0.5 * attitude_setpoints; // vector of half angle

//     Eigen::Vector3d omega = Eigen::Vector3d(bno_data->imu.angular_velocity.x, bno_data->imu.angular_velocity.y, bno_data->imu.angular_velocity.z);
//     auto quaternion_measured = Eigen::Quaterniond(bno_data->imu.orientation.w, bno_data->imu.orientation.x, bno_data->imu.orientation.y, bno_data->imu.orientation.z);
//     this->setpoint_mutex.unlock();
//     this->bno_mutex.unlock();

//     // update desired force
//     // Linear, discrete-time PID controller
//     // lambdas for linear PID loop
//     auto PID_params = this->get_parameters(std::vector<std::string>{"kp", "ki", "kd"});
//     auto P = [](rclcpp::Parameter kp, Eigen::Vector3d& linear_velocity_err)->Eigen::Vector3d {
//         return kp.as_double() * linear_velocity_err;
//     };
//     // TODO: anti-windup
//     auto I = [this](rclcpp::Parameter ki, Eigen::Vector3d& linear_velocity_err, int& dt_ms)->Eigen::Vector3d {
//         this->linear_integral += ki.as_double() * linear_velocity_err * dt_ms / 1000;
//         return this->linear_integral;
//     };
//     auto D = [](rclcpp::Parameter kd, Eigen::Vector3d& linear_velocity_err, Eigen::Vector3d& linear_velocity_err_last, int& dt_ms)->Eigen::Vector3d {
//         return kd.as_double() * (linear_velocity_err - linear_velocity_err_last) / (static_cast<double>(dt_ms) / 1000);
//     };

//     // P + I + D
//     desired_force = P(PID_params.at(0),linear_velocity_err)
//                     + I(PID_params.at(1), linear_velocity_err, dt_ms)
//                     + D(PID_params.at(2), linear_velocity_err, linear_velocity_err_last, dt_ms);

//     // update desired torque
//     // TODO: look at this if performance needs to be improved (loses accuracy tho)
//     // see https://stackoverflow.com/questions/24197182/efficient-quaternion-angular-velocity/24201879#24201879
//     double l = ha.norm(); // magnitude
//     if (l > 0) {
//         ha *= sin(l) / l;
//         quaternion_reference = Eigen::Quaterniond(cos(l), ha.x(), ha.y(), ha.z());
//     } else {
//         quaternion_reference = Eigen::Quaterniond(1.0, ha.x(), ha.y(), ha.z());
//     }
//     // desired orientation is only known by the pilot, quaternion reference defines a reference from a unity quaternion (1,0,0,0)
//     // to get actual reference, this must be multiplied with the measured quaternion to get desired orientation
//     // TODO: ensure that when w = 0, this is close to, or equal to unity
//     quaternion_reference = quaternion_reference * quaternion_measured;

//     // Non Linear P^2 Quaternion based control scheme
//     // For derivation of controller see: http://www.diva-portal.org/smash/get/diva2:1010947/FULLTEXT01.pdf
//     // For derivation of dynamics and control allocation see: https://flex.flinders.edu.au/file/27aa0064-9de2-441c-8a17-655405d5fc2e/1/ThesisWu2018.pdf
//     auto q_err = quaternion_reference * quaternion_measured.conjugate(); // hamilton product (hopefully)
//     Eigen::Vector3d axis_err;

//     if(q_err.w() < 0) { // this could be simplified to a negation but eh
//         axis_err = q_err.conjugate().vec();
//     } else {
//         axis_err = q_err.vec();
//     }

//     auto P2_params = this->get_parameters(std::vector<std::string>{"Pq", "Pw"});
//     desired_torque = (-P2_params.at(0).as_double() * axis_err) - (P2_params.at(1).as_double() * omega);

//     // control allocation
//     // u = K^-1 * T^+ * t
//     // 8x1 = 8x8 * 8x6 * 6x1 :)
//     Eigen::Matrix<double, 6, 1> forcesAndTorques;
//     forcesAndTorques << desired_force, desired_torque;

//     //TODO:test
//     // solve Ax = b and normalize thrust such that it satisfies MIN_THRUST_VALUE <= throttles[j] <= MAX_THRUST_VALUE 
//     // while scaling thrusters to account for large thrust demands on a single thruster
//     Eigen::Matrix<double, NUM_THRUSTERS, 1> actuations = this->thruster_coefficient_matrix_times_geometry * forcesAndTorques;
//     clampthrottles(&actuations);

//     // publish PWM values
//     // TODO: consider refactoring this to have 8 separate publishers?
//     // publishers would be created upon successful return from registerThrusters() if Use_PCA9685 == true
//     // else defaults would be created from params.yaml
//     // this method would allow for rqt_topic to more accurately collect PWM values
//     for(int i = 0; i < NUM_THRUSTERS; i++) {
//         rov_interfaces::msg::PWM msg;
//         msg.angle_or_throttle = static_cast<float>(actuations(i,0)); // this is a source of noise in output signals, may cause system instability??
//         msg.channel = thruster_index_to_PWM_pin.at(i);
//         pwm_publisher->publish(msg);
// #if DEBUG_OUTPUT
//         RCLCPP_DEBUG(this->get_logger(), "PIN:%i THROTTLE:%f", thruster_index_to_PWM_pin.at(i), actuations(i,0));
// #endif
//     }
// }

void FlightController::clampthrottles(Eigen::Matrix<double,NUM_THRUSTERS,1>* throttles) {
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


void FlightController::updateDepth(Eigen::Matrix<double, 6, 1>& desired_forces_torques) {
    const std::lock_guard<std::mutex> l1(bar30_mutex);
    const std::lock_guard<std::mutex> l2(bno_mutex);
    const Eigen::Vector3d sensor_offset(-0.36,0,0);
    float depth = tare_depth - bar30_data->depth;
    // correct for pitch
    Eigen::Quaterniond quat(bno_data->imu.orientation.w, bno_data->imu.orientation.x, bno_data->imu.orientation.y, bno_data->imu.orientation.z);
    Eigen::Vector3d grav(bno_data->gravity.i, bno_data->gravity.j, bno_data->gravity.k);
    depth += (quat * sensor_offset).dot(grav);
    float depth_error = depth_setpoint - depth;
    float kp = static_cast<float>(this->get_parameter("kpd").as_double());

    // we want to stabilize towards no depth delta between setpoint and where we are
    desired_forces_torques.segment<3>(0) += kp * depth_error * grav.normalized();
}

void FlightController::depthTare(const std_srvs::srv::Empty_Request::SharedPtr request, std_srvs::srv::Empty_Response::SharedPtr response) {
    (void) request;
    (void) response;
    const std::lock_guard<std::mutex> l1(bar30_mutex);
    const std::lock_guard<std::mutex> l2(bno_mutex);
    const Eigen::Vector3d sensor_offset(-0.36,0,0);
    // correct for pitch
    Eigen::Quaterniond quat(bno_data->imu.orientation.w, bno_data->imu.orientation.x, bno_data->imu.orientation.y, bno_data->imu.orientation.z);
    Eigen::Vector3d grav(bno_data->gravity.i, bno_data->gravity.j, bno_data->gravity.k);
    tare_depth = bar30_data->depth - (quat * sensor_offset).dot(grav);
}

void FlightController::updatePitchRoll(Eigen::Matrix<double, 6, 1>& desired_forces_torques) {
    const std::lock_guard<std::mutex> l1(bno_mutex);
    // remove impact from pitch and roll (!!!!!EULER WARNING!!!!!)
    Eigen::AngleAxisd yaw_only(bno_data->euler.k, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q_yaw_only(yaw_only);
    Eigen::Quaterniond quat(bno_data->imu.orientation.w, bno_data->imu.orientation.x, bno_data->imu.orientation.y, bno_data->imu.orientation.z);

    Eigen::Quaterniond q_err = q_yaw_only * quat.conjugate();
    Eigen::Vector3d angular_velocity(bno_data->imu.angular_velocity.x, bno_data->imu.angular_velocity.y, bno_data->imu.angular_velocity.z);
    desired_forces_torques.segment<3>(3) -= this->get_parameter("kppq").as_double() * q_err.vec() + this->get_parameter("kdpq").as_double() * angular_velocity;
}

void FlightController::updateStab() {
    Eigen::Matrix<double, 6, 1> forces_and_torques;

    // will attempt to drive orientation to horizontal unless disabled
    if (!free_orientation) {
        updatePitchRoll(forces_and_torques);
    }
    // TODO: will be a button on the controller that you press to activate depth controller
    if (keep_depth) {
        updateDepth(forces_and_torques);
    }

    //TODO:test
    // solve Ax = b with x = A^+ b and normalize thrust such that it satisfies MIN_THRUST_VALUE <= throttles[j] <= MAX_THRUST_VALUE 
    // while scaling thrusters to account for large thrust demands on a single thruster
    Eigen::Matrix<double, NUM_THRUSTERS, 1> actuations = this->thruster_coefficient_matrix_times_geometry * forces_and_torques;
    clampthrottles(&actuations);

    // publish PWM values
    // TODO: consider refactoring this to have 8 separate publishers?
    // publishers would be created upon successful return from registerThrusters() if Use_PCA9685 == true
    // else defaults would be created from params.yaml
    // this method would allow for rqt_topic to more accurately collect PWM values
    for(int i = 0; i < NUM_THRUSTERS; i++) {
        rov_interfaces::msg::PWM msg;
        msg.angle_or_throttle = static_cast<float>(actuations(i,0)); // this is a source of noise in output signals, may cause system instability??
        msg.channel = thruster_index_to_PWM_pin.at(i);
        pwm_publisher->publish(msg);
#if DEBUG_OUTPUT
        RCLCPP_DEBUG(this->get_logger(), "PIN:%i THROTTLE:%f", thruster_index_to_PWM_pin.at(i), actuations(i,0));
#endif
    }
}

