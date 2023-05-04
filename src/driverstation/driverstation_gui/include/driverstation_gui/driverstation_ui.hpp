#ifndef DRIVERSTATION_UI_HEADER_INCLUDED
#define DRIVERSTATION_UI_HEADER_INCLUDED

#include <cstring>
#include <functional>
#include <memory>
#include <sys/types.h>
#include <vector>

#define RAYGUI_IMPLEMENTATION
#include <raygui.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

#include "artificial_horizon_ui.hpp"
#include "camera_viewport_ui.hpp"
#include "config_main.hpp"
#include "dynamic_texture.hpp"
#include "estop_ui.hpp"
#include "orientation_viewport_ui.hpp"
#include "modes_util.hpp"
#include "ssh_ui.hpp"
#include "ui_util.hpp"
#include "horrible_ros_path.hpp"

namespace chosen_config = driverstation::configs::main;

namespace driverstation::gui{
	namespace{
		enum class SshConnectPopupMode {
			Host,
			Username,
			Password
		};
	}

	class DriverstationUi{
		public:
			DriverstationUi(int screen_width, int screen_height, const char* window_title, rclcpp::Node::SharedPtr node){
				this->node = node;
				InitWindow(screen_width, screen_height, window_title);
				SetTargetFPS(60);
				GuiLoadStyle(horrible_ros_path::prepend_prefix("assets/styles/dark.rgs").c_str());

				this->ssh_display = chosen_config::InitializeSshDisplay();
				this->ssh_connect_popup_mode = SshConnectPopupMode::Host;

				util::rendering::InitRendering();

				this->multi_cell = std::make_unique<dynamic_texture::MultiCellDynamicTextureGrid2D<12, 12>>(
					GetColor(GuiGetStyle(DEFAULT, BACKGROUND_COLOR)),
					screen_width, screen_height
				);

				this->camera_feeds[0] = std::make_shared<dynamic_texture::ImageFeed>();
				this->camera_feeds[1] = std::make_shared<dynamic_texture::ImageFeed>();
				this->camera_feeds[2] = std::make_shared<dynamic_texture::ImageFeed>();
				this->camera_feeds[3] = std::make_shared<dynamic_texture::ImageFeed>();

				auto camera_grid = std::make_shared<dynamic_texture::HomogeneousDynamicTextureGrid2D<gui::CameraViewport, 2, 2>>(1112, 834);
				for(uint8_t y = 0; y < 2; y++){
					for(uint8_t x = 0; x < 2; x++){
						auto camera = std::make_unique<gui::CameraViewport>(
							1112, 834,
							this->camera_feeds[y+y+x],
							WHITE,
							16
						);

						camera->label = "Camera";
						camera->unavailable_label = "Stream Unavailable";

						camera_grid->cells[y][x] = std::move(camera);
					}
				}

				uint32_t w, h;

				this->multi_cell
					// ->from(3, 0).to(9, 8).set(chosen_config::InitializeCameraGrid())
					->from(3, 0).to(9, 8).set(camera_grid)
					->from(9, 0).to(12, 6).tapSizePixels(&w, &h).set(std::make_shared<gui::OrientationViewport>("assets/rov.obj", w, h))
					->from(9, 6).to(12, 8).tapSizePixels(&w, &h).set(std::make_shared<gui::ArtificialHorizon>(w, h))
					->from(4, 8).to(8, 12).set(this->ssh_display)
					->from(11, 9).to(12, 11).tapSizePixels(&w, &h).set(std::make_shared<gui::EStop>(w, h, 2, std::bind(&DriverstationUi::callAllEstopCallbacks, this)));
			}

			~DriverstationUi(){
				util::rendering::UnloadRendering();

				CloseWindow();
			}

			bool refresh(){
				if(IsKeyReleased(KEY_F11)) ToggleFullscreen();

				// this->camera_grid_viewport->update(this->joystick_display->joystick_id);

				if(this->ssh_display->currentStage() == 4){  // Channel connected
					int key;
					while((key = GetKeyPressed()) > 0){this->ssh_display->sendKey(key);}

					// TODO: Temporary, make better
					if(IsKeyDown(KEY_RIGHT_ALT)){
						this->ssh_display->ssh_interface.closeSession();
						this->ssh_display->cursor_position = Vector2{0.0f, 0.0f};
						this->ssh_display->scroll_position = 0.0f;
						{const modes::TextureMode texture_mode(this->ssh_display->history_render_texture);
							ClearBackground(this->ssh_display->current_background_color);
						}
					}
				}

				{const modes::Drawing main_drawing;
					ClearBackground(GetColor(GuiGetStyle(DEFAULT, BACKGROUND_COLOR)));

					this->multi_cell->draw(0.0f, 0.0f);

					if(this->ssh_display->currentStage() < 3){  // Channel not connected
						char* text_target;
						switch(this->ssh_connect_popup_mode){
							case SshConnectPopupMode::Host:
								text_target = this->ssh_host;
								this->ssh_connect_popup_show_text = 1;
								break;
							case SshConnectPopupMode::Username:
								text_target = this->ssh_username;
								this->ssh_connect_popup_show_text = 1;
								break;
							case SshConnectPopupMode::Password:
								text_target = this->ssh_password;
								this->ssh_connect_popup_show_text = 0;
								break;
						}

						int response = GuiTextInputBox(
							this->multi_cell->bounds_of(this->ssh_display).value(),
							"Connect to SSH",
							TextFormat("Host: %s\nUsername: %s\nPassword:", this->ssh_host, this->ssh_username),
							"Set Host;Set Username;Set Password;Connect",
							text_target, 32, &this->ssh_connect_popup_show_text);
						
						switch(response){
							case 1:
								this->ssh_connect_popup_mode = SshConnectPopupMode::Host;
								break;
							case 2:
								this->ssh_connect_popup_mode = SshConnectPopupMode::Username;
								break;
							case 3:
								this->ssh_connect_popup_mode = SshConnectPopupMode::Password;
								break;
							case 4:
								// TODO: Handle errors?
								this->ssh_display->connectTo(this->ssh_host, this->ssh_username, this->ssh_password);
								break;
						}
					}

					if(IsMouseButtonPressed(MOUSE_LEFT_BUTTON)){
						auto position = GetMousePosition();
						auto [w, h] = this->multi_cell->size();

						this->multi_cell->on_click(position.x / w, position.y / h);
					}

					driverstation::util::measurement::HandleRectVisualization();

					DrawFPS(1800, 20);
				}

				return !WindowShouldClose();
			}

			void setCameraImage(uint camera_index, uint width, uint height, const void* frame_data){
				if(camera_index < 4){
					this->camera_feeds[camera_index]->setImage(width, height, frame_data);
				}
			}

			void registerEstopCallback(std::function<void(void)> callback){
				this->estop_callbacks.push_back(callback);
			}

		protected:
			rclcpp::Node::SharedPtr node;
			// rclcpp::Publisher<>::SharedPtr pub;
			
			std::unique_ptr<dynamic_texture::MultiCellDynamicTextureGrid2D<12, 12>> multi_cell;
			std::shared_ptr<gui::SshDisplay> ssh_display;
			// TODO: FIXME: ssh_connect_popup_mode type uses anonymous namespace and is in header file
			SshConnectPopupMode ssh_connect_popup_mode;
			char ssh_host[32];
			char ssh_username[32];
			char ssh_password[32];
			int ssh_connect_popup_show_text;
			std::shared_ptr<dynamic_texture::ImageFeed> camera_feeds[4];
			std::vector<std::function<void(void)>> estop_callbacks;
		
		private:
			void callAllEstopCallbacks(){
				for(auto callback : this->estop_callbacks){
					callback();
				}
			}
	};
}

#endif
