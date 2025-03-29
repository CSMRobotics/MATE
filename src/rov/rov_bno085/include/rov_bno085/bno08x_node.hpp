#ifndef BNO08X_NODE_HPP
#define BNO08X_NODE_HPP

#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_srvs/srv/empty.hpp>
// TODO include quat definition or just vectord?

#include "rov_bno085/bno08x_driver.hpp"

class BNO08X_Node : public rclcpp::Node {
public:
    BNO08X_Node(uint8_t bus = 1);
    BNO08X_Node(const BNO08X_Node&) = delete;
    BNO08X_Node(BNO08X_Node&&) = default;
    ~BNO08X_Node();

    void open();
    void close();
    void reset();

private:
    void pollBNO();
    void publishBNOData();

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_accel_gyro;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_temp; 

    rclcpp::TimerBase::SharedPtr timer_poll_bno;

    BNO08X bno08x;

    int spi_fd;
};

#endif