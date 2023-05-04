#include <rclcpp/rclcpp.hpp>
#include "driverstation_gui/driverstation_ui.hpp"

int main(int argc, char** argv){
	rclcpp::init(argc, argv);

	rclcpp::executors::SingleThreadedExecutor exec;
	rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("driverstation_gui");
	exec.add_node(node);

	auto driverstation_ui = std::make_unique<driverstation::gui::DriverstationUi>(1920, 1080, "Driverstation GUI", node);
	while(driverstation_ui->refresh()) {
		exec.spin_all(std::chrono::milliseconds(1000/60));
	}

	rclcpp::shutdown();
	return 0;
}
