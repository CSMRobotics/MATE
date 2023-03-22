#include <rclcpp/rclcpp.hpp>
#include "driverstation_gui/driverstation_ui.hpp"

int main(int argc, char** argv){
	rclcpp::init(argc, argv);

	auto driverstation_ui = std::make_unique<driverstation::gui::DriverstationUi>(1920, 1080, "Driverstation GUI");
	while(driverstation_ui->refresh());

	rclcpp::shutdown();
	return 0;
}
