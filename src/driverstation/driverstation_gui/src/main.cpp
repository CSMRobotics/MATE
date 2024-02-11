#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include "driverstation_gui/driverstation_ui.hpp"

int main(int argc, char** argv){
	rclcpp::init(argc, argv);

	rclcpp::executors::SingleThreadedExecutor exec;
	rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("driverstation_gui");
	exec.add_node(node);

	auto driverstation_ui = std::make_unique<driverstation::gui::DriverstationUi>(1920, 1080, "Driverstation GUI", node);
	bool estop_triggered = false;

	auto image_subscriber = node->create_subscription<sensor_msgs::msg::CompressedImage>(
		"camera_mjpeg0/image_raw/compressed", 10,
		[&](sensor_msgs::msg::CompressedImage::SharedPtr image_message){
			Image image = LoadImageFromMemory(".png", (unsigned char*)image_message->data.data(), image_message->data.size());
			driverstation_ui->setCameraImage(0, image);
			UnloadImage(image);
		}
	);

	driverstation_ui->registerEstopCallback([&](){
		estop_triggered = true;
	});

	while(!estop_triggered && rclcpp::ok() && driverstation_ui->refresh()) {
		exec.spin_all(std::chrono::milliseconds(1000/60));
	}

	rclcpp::shutdown();
	return 0;
}
