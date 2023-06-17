#include "flight_controller/controller.hpp"

#define DEBUG_OUTPUT true
#include <sstream>

// various anonymous functions
namespace {
void clampthrottles(Eigen::Matrix<double,NUM_THRUSTERS,1>* const throttles) {
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

void declare_flight_controller_parameters(rclcpp::Node*const node) {
    // declare thruster parameters
    std::stringstream ss;
    std::string s;
    for(int i = 0; i < NUM_THRUSTERS; i++) {
        ss.str("");
        ss << "thruster";
        ss << i;
        // default position and thrust vectors to 0
        node->declare_parameters(ss.str(), std::map<std::string, std::vector<double>>{
            {"position", std::vector<double>{0,0,0}},
            {"thrust", std::vector<double>{0,0,0}} // unit vector
        });
        // default PWM pin to i in [0,NUM_THRUSTERS]
        node->declare_parameters(ss.str(), std::map<std::string, int>{
            {"Pin", i}
        });
    }

    node->declare_parameter("kp", 1.0);
    node->declare_parameter("kq", 1.0);
    node->declare_parameter("kw", 1.0);
    node->declare_parameter("kdp", 1.0);
    node->declare_parameter("kdi", 0.1);
    node->declare_parameter("depth_tare", 0.0);
    node->declare_parameter("depth_integral_max_contrib", 0.2);
    node->declare_parameter("max_depth_m", 6.0);
    node->declare_parameter("mass_kg", 1.0);
    node->declare_parameter("volume_m3", 1.0);
    node->declare_parameter("cg", std::vector<double>{0.0, 0.0, 0.0});
    node->declare_parameter("cb", std::vector<double>{0.0, 0.0, 0.0});
}

// Parse the given Flight Controller's thruster<idx> parameters and apply them to supplied containers
void initialize_thrusters(FlightController* const node, std::array<Thruster, NUM_THRUSTERS>* const thrusters ,std::unordered_map<int,int>* const thruster_idx_to_pwm_pin, Eigen::Matrix<double, 6, NUM_THRUSTERS>* const thruster_geometry) {
    std::stringstream ss;
    std::array<Eigen::Matrix<double, 6, 1>, NUM_THRUSTERS> temp;
    for(int i = 0; i < NUM_THRUSTERS; i++) {
        ss.str("");
        ss << "thruster" << i << '.';
        // Construct internal thruster data type from ROS parameter
        Thruster t = {
            Eigen::Vector3d(node->get_parameter(ss.str()+"position").as_double_array().data()),
            Eigen::Vector3d(node->get_parameter(ss.str()+"thrust").as_double_array().data()),
            node->get_parameter(ss.str()+"pin").as_int()
        };

        (*thrusters)[i] = t;
        // calculate linear and rotation contribution
        Eigen::Vector3d linear_contribution(t.thrust);
        // TODO: accurately get position of thrusters
        Eigen::Vector3d rotation_contribution(t.position.cross(t.thrust));

        // add the PWM pin to the thruster index map
        thruster_idx_to_pwm_pin->emplace(std::make_pair(i, t.pwm_pin));
        #if DEBUG_OUTPUT
            RCLCPP_DEBUG(node->get_logger(), "Thruster %i: linear: %f, %f, %f  rotation: %f, %f, %f\n",
                i,
                linear_contribution(0,0),
                linear_contribution(1,0),
                linear_contribution(2,0),
                rotation_contribution(0,0),
                rotation_contribution(1,0),
                rotation_contribution(2,0));
        #endif

        // add the thruster's geometry to the thruster geometry matrix
        temp[i] = Eigen::Matrix<double, 6, 1>();
        temp[i] << linear_contribution, rotation_contribution;
        thruster_geometry->col(i) << temp[i];
    }
}

inline Eigen::Matrix3d skew_symmetric(Eigen::Vector3d lambda) {
    Eigen::Matrix3d ret;
    ret << 0.0, -lambda.z(), lambda.y(), lambda.z(), 0.0, -lambda.x(), -lambda.y(), lambda.x(), 0.0;
    return ret;
}

[[maybe_unused]] void fuzzy_linear_velocity(Eigen::Vector3d& velocity) {
    if(velocity.isMuchSmallerThan(VELOCITY_PREC))
        velocity = Eigen::Vector3d::Zero();
}

}

FlightController::FlightController() : rclcpp::Node("flight_controller") {
    // declare this node's parameters
    declare_flight_controller_parameters(this);

    // parse config file's thruster definitions
    initialize_thrusters(this, &this->thrusters, &this->thruster_idx_to_pwm_pin, &this->thruster_geometry);

    // create ros pubs and subs
    this->bar_sub = this->create_subscription<rov_interfaces::msg::Bar02Data>("bar02_data", rclcpp::SensorDataQoS(), std::bind(&FlightController::bar_callback, this, std::placeholders::_1));
    this->bno_sub = this->create_subscription<rov_interfaces::msg::BNO055Data>("bno055_data", rclcpp::SensorDataQoS(), std::bind(&FlightController::bno_callback, this, std::placeholders::_1));
    this->thruster_setpoint_sub = this->create_subscription<rov_interfaces::msg::ThrusterSetpoints>("thruster_setpoints", 10, std::bind(&FlightController::thruster_setpoint_callback, this, std::placeholders::_1));
    this->depth_setpoint_sub = this->create_subscription<std_msgs::msg::Float64>("depth_setpoint", 10, std::bind(&FlightController::depth_setpoint_callback, this, std::placeholders::_1));
    this->estop_sub = this->create_subscription<csm_common_interfaces::msg::EStop>("estop", 10, std::bind(&FlightController::estop_callback, this, std::placeholders::_1));

    this->pwm_pub = this->create_publisher<rov_interfaces::msg::PWM>("pwm", 10);

    // compute the pseudo inverse
    // The columns of thruster_geometry should be linearly independent
    assert(thruster_geometry.fullPivLu().kernel().isZero());
    thruster_geometry_left_pseudoinverse = (thruster_geometry.transpose() * thruster_geometry).inverse() * thruster_geometry.transpose();
    #if DEBUG_OUTPUT
        RCLCPP_INFO(this->get_logger(), "Is the Null Space of thruster geometry 0? %d", thruster_geometry.fullPivLu().kernel().isZero());
    #endif

    // initialize I
    // TODO: parameterize
    I = Eigen::Matrix3d::Zero();

    // initialize M_rb
    M_rb = Eigen::Matrix<double, 6, 6>::Zero();
    M_rb.block<3,3>(0,0) << Eigen::DiagonalMatrix<double, 3>(Eigen::Vector3d::Ones() * this->get_parameter("mass_kg").as_double()).toDenseMatrix();
    M_rb.block<3,3>(0, 3) = -this->get_parameter("mass_kg").as_double() * skew_symmetric(Eigen::Vector3d(this->get_parameter("cg").as_double_array().data()));
    M_rb.block<3,3>(3, 0) = M_rb.block<3,3>(0, 3).transpose();
    M_rb.block<3,3>(3,3) = I;
    #if DEBUG_OUTPUT
        std::stringstream ss;
        ss << M_rb;
        RCLCPP_INFO(this->get_logger(), "M_rb was set to\n%s", ss.str().c_str());
    #endif

    // initialize C_rb
    C_rb = Eigen::Matrix<double, 6, 6>::Zero();

    // TODO: experimentally determine
    // initialize D
    D = Eigen::Matrix<double, 6, 6>::Zero();

    // start ros update loop
    this->update_timer = this->create_wall_timer(std::chrono::milliseconds(1000/UPDATE_RATE_HZ), std::bind(&FlightController::startup_delay_callback, this));
    this->last_updated = std::chrono::high_resolution_clock::now();
}

void FlightController::startup_delay_callback() {
    static bool is_not_first = false;
    static std::chrono::time_point<std::chrono::high_resolution_clock> tp;
    if(!is_not_first) {is_not_first = true; tp = std::chrono::high_resolution_clock::now() + std::chrono::seconds(2);}
    if(std::chrono::high_resolution_clock::now() >= tp) {
        this->update_timer = this->create_wall_timer(std::chrono::milliseconds(1000/UPDATE_RATE_HZ), std::bind(&FlightController::update_callback, this));
    }
}

void FlightController::update_callback() {
    // call the desired update function
    { std::lock_guard lock(update_mutex);
    desired_update(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - last_updated).count());
    }

    // scale actuations by abs of largest actuation request
    clampthrottles(&actuations);

    // publish PWM values to apply the actuations to the thrusters
    { std::lock_guard lock(pwm_mutex);
    for(int i = 0; i < NUM_THRUSTERS; i++) {
        rov_interfaces::msg::PWM msg;
        msg.angle_or_throttle = static_cast<float>(actuations(i,0)); // yeah we convert from a double to a float :(
        msg.channel = thruster_idx_to_pwm_pin.at(i);
        pwm_pub->publish(msg);
        #if DEBUG_OUTPUT
            RCLCPP_INFO(this->get_logger(), "PIN:%i THROTTLE:%f", thruster_idx_to_pwm_pin.at(i), actuations(i,0));
        #endif
    }
    }

    last_updated = std::chrono::high_resolution_clock::now();
}

void FlightController::update_simple([[maybe_unused]] long dt_ns) {
    #if DEBUG_OUTPUT
        RCLCPP_DEBUG(this->get_logger(), "Simple Update Called");
    #endif
    thruster_setpoint_data setpoints_CPY;
    { std::lock_guard lock(setpoint_mutex);
    setpoints_CPY = thruster_setpoints;
    }

    // convert setpoints to actuations that the controller must apply
    for(int i = 0; i < NUM_THRUSTERS; i++) {
        Thruster t = thrusters[i];
        // Actuation = Linear Actuation(0) + Rotational Actuation (0)
        // ALTERNATIVELY: G^+ * tau = activation for rotation or translation defined but not both
        actuations(i,0) = t.thrust.dot(setpoints_CPY.translational)
            + t.position.cross(t.thrust).dot(setpoints_CPY.rotational);
    }
}

void FlightController::update_linear_PID([[maybe_unused]] long dt_ns) {
    #if DEBUG_OUTPUT
        RCLCPP_DEBUG(this->get_logger(), "Linear PID Update Called");
    #endif
}

void FlightController::update_non_linear_PID([[maybe_unused]] long dt_ns) {
    static Eigen::Quaterniond quaternion_reference;
    static Eigen::Vector3d ha;
    static Eigen::Vector3d omega;
    static Eigen::Quaterniond quaternion_measured;

    // update translations aswell
    update_linear_PID(dt_ns);

    { std::lock_guard lock(setpoint_mutex); std::lock_guard lock2(bno_mutex);
    ha = dt_ns * NS_TO_S * 0.5 * thruster_setpoints.rotational; // vector of half angle
    omega = imu_data.gyroscope;
    quaternion_measured = imu_data.quaternion;
    }

    // update desired torque
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

    auto P2_params = this->get_parameters(std::vector<std::string>{"Pq", "Pw"});
    desired_moment = (-P2_params.at(0).as_double() * axis_err) - (P2_params.at(1).as_double() * omega);
}

void FlightController::update_depth_controller_PID(long dt_ns) {
    static double depth_error_last = 0;
    static double integral_contrib = 0;

    // pressure setpoints, pressure data, and IMU data
    depth_setpoint_data setpoint_cpy;
    { std::lock_guard lock(setpoint_mutex);
        setpoint_cpy = depth_setpoints;
    }
    bar_data bar_data_cpy;
    { std::lock_guard lock(bar_mutex);
        bar_data_cpy = depth_data;
    }
    bno_data bno_data_cpy;
    { std::lock_guard lock(bno_mutex);
        bno_data_cpy = imu_data;
    }

    // calculate depth error
    // TODO: implement service to set depth_tare to current bar_data depth
    double depth_error = setpoint_cpy.depth_m - bar_data_cpy.depth + this->get_parameter("depth_tare").as_double();
    double depth_error_add = depth_error + depth_error_last;
    depth_error_last = depth_error;

    // get gain constants
    double kdp = this->get_parameter("kdp").as_double();
    double kdi = this->get_parameter("kdi").as_double();
    double integral_max_contrib = this->get_parameter("depth_integral_max_contrib").as_double();

    // P and I controller definitions
    auto P = [&]() {
        return kdp * depth_error;
    };

    auto I = [&]() {
        // with anti windup
        return integral_contrib = std::min(integral_contrib + kdi * (depth_error_add) / 2 * dt_ns * NS_TO_S, integral_max_contrib);
    };

    // transform thruster geometry from body to ned
    // update Jq
    update_body_to_ned_transform(bno_data_cpy.quaternion);
    // do the transform
    Eigen::Matrix<double, 7, NUM_THRUSTERS> thruster_geometry_ned = Jq * thruster_geometry;
    
    // apply controller output using method similar to UpdateSimple
    // since we want to maintain normal control authority, call update_simple before fusing this controller's output
    update_simple(dt_ns);
    Eigen::Vector3d controller_output = (P() + I()) * Eigen::Vector3d{0.0, 0.0, 1.0};
    for(int i = 0; i < NUM_THRUSTERS; i++) {
        actuations(i,0) = Eigen::Vector3d(thruster_geometry_ned.block<3,1>(0, i)).dot(controller_output);
    }
}

void FlightController::update_coriolis_centripetal_matrix(const Eigen::Vector3d& v, const Eigen::Vector3d& w) {
    double m = this->get_parameter("mass_kg").as_double();
    Eigen::Vector3d xyz_G = Eigen::Vector3d(this->get_parameter("cg").as_double_array().data());
    // see page (3.56) on page 69 of Thor I. Fossen - Handbook of Marine Craft Hydrodynamics and Motion Control (2011) 
    C_rb.block<3,3>(0,3) = -m * skew_symmetric(v) - m * skew_symmetric(w) * skew_symmetric(xyz_G);
    C_rb.block<3,3>(3,0) = -m * skew_symmetric(v) + m * skew_symmetric(xyz_G) * skew_symmetric(w);
    C_rb.block<3,3>(3,3) = -skew_symmetric(I * w);
}

void FlightController::update_hydrodynamic_damping_matrix([[maybe_unused]] const Eigen::Vector3d& d, [[maybe_unused]] const Eigen::Vector3d& v, [[maybe_unused]] const Eigen::Vector3d& w) {

}

void FlightController::update_gravitational_and_buoyancy_vector(const Eigen::Vector3d& euler) {
    constexpr double EPSILON = 1e-3;
    double W = this->get_parameter("mass_kg").as_double() * 9.81;
    double B = 1000 * 9.81 * this->get_parameter("volume_m3").as_double();
    double W_B = W-B;
    Eigen::Vector3d xyz_B = Eigen::Vector3d(this->get_parameter("cb").as_double_array().data());
    Eigen::Vector3d xyz_G = Eigen::Vector3d(this->get_parameter("cg").as_double_array().data());
    Eigen::Vector3d BG = xyz_G-xyz_B;
    if(std::abs(W_B) > EPSILON) { // if the weight and buoyancy differ by 0.001 N rov is not neutrally buoyant, more math required
        g_res.block<5,1>(0,0) <<    W_B * sinf64(euler.y()), 
                                    -(W_B)*cosf64(euler.y())*sinf64(euler.x()), 
                                    -(W_B)*cosf64(euler.y())*cosf64(euler.x()),
                                    -(xyz_G.y()*W - xyz_B.y()*B)*cosf64(euler.y())*cosf64(euler.x()) + (xyz_G.z()*W - xyz_B.z()*B)*cosf64(euler.y())*sinf64(euler.x()),
                                    (xyz_G.z()*W - xyz_B.z()*B)*sinf64(euler.y())                   + (xyz_G.x()*W - xyz_B.x()*B)*cosf64(euler.y())*cosf64(euler.x()),
                                    -(xyz_G.x()*W - xyz_B.x()*B)*cosf64(euler.y())*sinf64(euler.x()) - (xyz_G.y()*W - xyz_B.y()*B)*sinf64(euler.y());
    } else {
        g_res.block<5,1>(0,0) <<    0.0,
                                    0.0,
                                    0.0,
                                    -BG.y()*W*cosf64(euler.y())*cosf64(euler.x()) + BG.z()*W*cosf64(euler.y())*sinf64(euler.x()),
                                    BG.z()*W*sinf64(euler.y())                    + BG.x()*W*cosf64(euler.y())*cosf64(euler.x()),
                                    -BG.x()*W*cosf64(euler.y())*sinf64(euler.x()) - BG.y()*W*sinf64(euler.y());
    }
}

// WARNING: unverified
// See equation (4.5) on page 60 of Thor I. Fossen - Handbook of Marine Craft Hydrodynamics and Motion Control (2011)
void FlightController::update_gravitational_and_buoyancy_vector(const Eigen::Quaterniond& quat) {
    // get ROV parameters
    Eigen::Vector3d W = Eigen::Vector3d(0.0, 0.0, this->get_parameter("mass_kg").as_double() * 9.81);
    Eigen::Vector3d B = Eigen::Vector3d(0.0, 0.0, 1000 * 9.81 * this->get_parameter("volume_m3").as_double());
    Eigen::Vector3d xyz_B = Eigen::Vector3d(this->get_parameter("cb").as_double_array().data());
    Eigen::Vector3d xyz_G = Eigen::Vector3d(this->get_parameter("cg").as_double_array().data());

    // calculate velocity transformation matrix in terms of quat
    Eigen::Matrix3d R_qbton = I + 2*quat.w()*skew_symmetric(quat.vec()) + 2 * skew_symmetric(quat.vec()) * skew_symmetric(quat.vec());
    // calculate angular velocity transformation matrix in terms of quat
    // Eigen::Matrix<double, 4, 3> T_qofq;
    // T_qofq << - quat.vec().transpose(), quat.w()*I + skew_symmetric(quat.vec());
    // T_qofq *= 0.5;
    g_res.block<3, 1>(0,0) << R_qbton.inverse() * (W-B);
    g_res.block<3, 1>(3,0) << (xyz_G-xyz_B).cross(R_qbton * W);
    g_res = -g_res;
}

void FlightController::update_body_to_ned_transform(const Eigen::Quaterniond& quat) {
    // BL and TR never change from 0s
    // Update R_bn(q)
    Jq.block<3,3>(0,0) << I + 2 * quat.w() * skew_symmetric(quat.vec()) + 2 * skew_symmetric(quat.vec()) * skew_symmetric(quat.vec());
    // Update T_q(q)
    Jq.block<4,3>(3, 3) << -0.5 * quat.vec().transpose(), quat.w() * I + skew_symmetric(quat.vec());
}

void FlightController::estop_callback([[maybe_unused]] csm_common_interfaces::msg::EStop::SharedPtr msg) {
    estop_handler(true);
}

void FlightController::estop_handler(bool is_estop_call) {
    static bool ESTOP = false;
    static bool RESTARTED = false;
    static std::unique_lock lock_pwm{this->pwm_mutex, std::defer_lock};
    static std::unique_lock lock_update{this->update_mutex, std::defer_lock};
    static std::function<void(long)> store_func = std::bind(&FlightController::update_none, this, std::placeholders::_1);
    
    // if this is an ESTOP call, and we haven't processed an ESTOP yet, do so
    if(is_estop_call && !ESTOP) {
        ESTOP = true;
        RESTARTED = false;
        std::lock(lock_pwm, lock_update);
        std::swap(store_func, this->desired_update);
        
        // zero all thrusters
        for(int i=0; i < NUM_THRUSTERS; i++) {
            auto msg = rov_interfaces::msg::PWM();
            msg.channel = thruster_idx_to_pwm_pin.at(i);
            msg.angle_or_throttle = 0;
            pwm_pub->publish(msg);
        }
    }
    // if this is a restart call, unlock
    if(!is_estop_call && !RESTARTED) {
        ESTOP = false;
        RESTARTED = true;
        this->last_updated = std::chrono::high_resolution_clock::now();
        lock_pwm.unlock();
        lock_update.unlock();
        std::swap(store_func, this->desired_update);
    }
}

void FlightController::restart_after_estop([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty_Request> req, [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty_Response> res) {
    estop_handler(false);
}

void FlightController::thruster_setpoint_callback(rov_interfaces::msg::ThrusterSetpoints::SharedPtr msg) {
    auto lock = std::lock_guard(this->setpoint_mutex);
    thruster_setpoints = thruster_setpoint_data(msg);
}

void FlightController::depth_setpoint_callback(std_msgs::msg::Float64::SharedPtr msg) {
    auto lock = std::lock_guard(this->setpoint_mutex);
    depth_setpoints = depth_setpoint_data(msg, this->get_parameter("max_depth_m").as_double());
}

void FlightController::bar_callback(rov_interfaces::msg::Bar02Data::SharedPtr msg) {
    auto lock = std::lock_guard(this->bar_mutex);
    depth_data = bar_data(msg);
}

void FlightController::bno_callback(rov_interfaces::msg::BNO055Data::SharedPtr msg) {
    auto lock = std::lock_guard(this->bno_mutex);
    imu_data = bno_data(msg);   
}

bar_data::bar_data(rov_interfaces::msg::Bar02Data::SharedPtr msg) :
    altitude(msg->altitude), 
    depth(msg->depth), 
    pressure_kpa(msg->pressure), 
    temperature_c(msg->temperature_c) {
}

bno_data::bno_data(rov_interfaces::msg::BNO055Data::SharedPtr msg) :
    quaternion(msg->orientation.w, msg->orientation.i, msg->orientation.j, msg->orientation.k),
    accelerometer(msg->accelerometer.i, msg->accelerometer.j, msg->accelerometer.k),
    magnetometer(msg->magnetometer.i, msg->magnetometer.j, msg->magnetometer.k),
    gyroscope(msg->gyroscope.i, msg->gyroscope.j, msg->gyroscope.k),
    euler(msg->euler.i, msg->euler.j, msg->euler.k),
    linearaccel(msg->linearaccel.i, msg->linearaccel.j, msg->linearaccel.k),
    gravity(msg->gravity.i, msg->gravity.j, msg->gravity.k),
    temp(msg->temp),
    calibration(msg->calibration) {
}

thruster_setpoint_data::thruster_setpoint_data(rov_interfaces::msg::ThrusterSetpoints::SharedPtr msg) :
    translational(msg->vx, msg->vy, msg->vz),
    rotational(msg->omegax, msg->omegay, msg->omegaz) {
}

depth_setpoint_data::depth_setpoint_data(std_msgs::msg::Float64::SharedPtr msg, double max_depth_m) {
    this->depth_m = (msg->data += 1) * max_depth_m; // go from [-1, 1] -> [0, MAX_DEPTH]
}