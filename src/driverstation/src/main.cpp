#include "driverstation_ui.hpp"

int main(){
	auto driverstation_ui = std::make_unique<driverstation::gui::DriverstationUi>(1920, 1080, "Driverstation GUI");
	while(driverstation_ui->refresh());
	return 0;
}
