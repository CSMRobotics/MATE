#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
using std::placeholders::_1;

class JoystickSubscriber : public rclcpp::Node
{
  public:
    JoystickSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "topic", 10, std::bind(&JoystickSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const
    {
        
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoystickSubscriber>());
  rclcpp::shutdown();
  return 0;
}