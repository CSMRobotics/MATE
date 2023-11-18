#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "rov_interfaces/msg/bno055_data.hpp"

#include "geometry_msgs/msg/vector3.h"

#include "bno055/CSMBNO055.hpp"

using namespace std::chrono_literals;

#define max_attempts 5

// create cursed functions only available in this translation unit
namespace
{
    rov_interfaces::msg::VectorD toVectorDMSG(const Eigen::Vector3d vector)
    {
        auto toret = rov_interfaces::msg::VectorD();
        toret.i = vector.x();
        toret.j = vector.y();
        toret.k = vector.z();
        return toret;
    }
    /*
    rov_interfaces::msg::QuaternionD toQuatDMSG(const Eigen::Quaterniond quat) {
        auto toret = rov_interfaces::msg::QuaternionD();
        toret.w = quat.w();
        toret.i = quat.x();
        toret.j = quat.y();
        toret.k = quat.z();
        return toret;
    }
    */
}

// its a bno055 using i2c, its a ros2 node :o
class BNO055_Node : public rclcpp::Node
{
public:
    BNO055_Node() : Node("bno055_node"), bno(-1, BNO055_ADDRESS_A)
    {
        this->bno_publisher = this->create_publisher<rov_interfaces::msg::BNO055Data>("bno055_data", rclcpp::SensorDataQoS());
        
        //use parameter "update_ms" to control the update rate of the BNO055 reporting
        this->declare_parameter("update_ms", 100);
        param_handle = std::make_shared<rclcpp::ParameterEventHandler>(this);
        auto update_ms_callback = [&](const rclcpp::Parameter& param){
            if (param.as_int() > 0)
                bno_timer = this->create_wall_timer(std::chrono::milliseconds(param.as_int()), std::bind(&BNO055_Node::bno_callback, this));
        };
        bno_timer = this->create_wall_timer(std::chrono::milliseconds(this->get_parameter("update_ms").as_int()), std::bind(&BNO055_Node::bno_callback, this));
        callback_handle = param_handle->add_parameter_callback("update_ms", update_ms_callback);  
        
        bno.begin();
    }

private:
    // Create and send new BNO055 Data msg
    void bno_callback()
    {
        uint8_t sys, gyro, accel, mag;
        uint8_t calib;
        geometry_msgs::msg::Vector3 magnetometer;

        auto msg = rov_interfaces::msg::BNO055Data();

        // acceleration, euler, and gravity all use vectorD, so translate them using toVectorDMSG
        msg.accel = toVectorDMSG(this->bno.getVector(Vector_Type::ACCELEROMETER));
        msg.euler = toVectorDMSG(this->bno.getVector(Vector_Type::EULER));
        msg.gravity = toVectorDMSG(this->bno.getVector(Vector_Type::GRAVITY));

        // angular velocity uses x, y, and z
        msg.imu.angular_velocity.x = this->bno.getVector(Vector_Type::GYROSCOPE).x();
        msg.imu.angular_velocity.y = this->bno.getVector(Vector_Type::GYROSCOPE).y();
        msg.imu.angular_velocity.z = this->bno.getVector(Vector_Type::GYROSCOPE).z();

        msg.imu.linear_acceleration.x = this->bno.getVector(Vector_Type::LINEARACCEL).x();
        msg.imu.linear_acceleration.y = this->bno.getVector(Vector_Type::LINEARACCEL).y();
        msg.imu.linear_acceleration.z = this->bno.getVector(Vector_Type::LINEARACCEL).z();

        // return orientation to ros2 imu message
        //  .coeffs() is [x, y, z, w]
        msg.imu.orientation.w = this->bno.getQuat().coeffs().w();
        msg.imu.orientation.x = this->bno.getQuat().coeffs().x();
        msg.imu.orientation.y = this->bno.getQuat().coeffs().y();
        msg.imu.orientation.z = this->bno.getQuat().coeffs().z();

        // magnetometer needs a vector3, values cannot be manually assigned
        magnetometer.x = this->bno.getVector(Vector_Type::MAGNETOMETER).x();
        magnetometer.y = this->bno.getVector(Vector_Type::MAGNETOMETER).y();
        magnetometer.z = this->bno.getVector(Vector_Type::MAGNETOMETER).z();
        msg.magnetometer.set__magnetic_field(magnetometer);

        // prepare temperature message
        msg.temp.temperature = this->bno.getTemp();

        // prepare calibration data from bno055
        this->bno.getCalibration(&sys, &gyro, &accel, &mag);
        calib = sys;
        calib = (calib << 2) | gyro;
        calib = (calib << 2) | accel;
        calib = (calib << 2) | mag;
        msg.calibration = calib;
        this->bno_publisher->publish(msg);
    }

    BNO055 bno;
    std::shared_ptr<rclcpp::Publisher<rov_interfaces::msg::BNO055Data>> bno_publisher;
    rclcpp::TimerBase::SharedPtr bno_timer;
    std::shared_ptr<rclcpp::ParameterEventHandler> param_handle;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> callback_handle;
};

int main(int argc, char **argv)
{
    
    rclcpp::init(argc, argv);
    int current_attempt = 0;
    
    //retry spinning with exponential backoff, hardcoded to 5 for now 
    while(rclcpp::ok() && current_attempt < max_attempts){
        try{
            rclcpp::spin(std::make_shared<BNO055_Node>());   
        } catch(std::runtime_error& e){
            RCLCPP_FATAL(rclcpp::get_logger("bno055_node"), "Failed to open i2c device on bus");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000*(++current_attempt*2)));
        } catch(std::domain_error& e){
            RCLCPP_FATAL(rclcpp::get_logger("bno055_node"), "Failed to open I2C bus file");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000*(current_attempt*2)));
            current_attempt++;
        }
    }
    
    rclcpp::shutdown();
    return 0;
}
