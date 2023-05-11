#ifndef CONFIG_MAIN_HEADER_INCLUDED
#define CONFIG_MAIN_HEADER_INCLUDED

#include <array>
#include <raylib.h>

#include "artificial_horizon_ui.hpp"
#include "camera_viewport_ui.hpp"
#include "dynamic_texture.hpp"
#include "joystick_ui.hpp"
#include "orientation_viewport_ui.hpp"
#include "ssh_ui.hpp"
#include "ui_util.hpp"
#include "horrible_ros_path.hpp"

namespace driverstation::configs::main{
	const uint8_t CAMERA_GRID_ROW_COUNT = 2;
	const uint8_t CAMERA_GRID_COLUMN_COUNT = 2;
	const char* JOYSTICK_MAPPINGS = "030000006d04000015c2000011010000,Logitech Extreme 3D pro,platform:Linux,a:b6,b:b5,x:b7,y:b4,leftshoulder:b8,rightshoulder:b10,dpup:b0,dpdown:b2,dpleft:b3,dpright:b1,leftx:a0,lefty:a1,rightx:a2,righty:a3~,lefttrigger:b9,righttrigger:b11,";

	// TODO: Should this be templated over the grid size?
	// template<uint8_t row_count, uint8_t column_count>
	std::shared_ptr<dynamic_texture::HomogeneousDynamicTextureGrid2D<gui::CameraViewport, CAMERA_GRID_ROW_COUNT, CAMERA_GRID_COLUMN_COUNT>> InitializeCameraGrid(){
		auto fallback = std::make_shared<dynamic_texture::GifTexture2D>(horrible_ros_path::prepend_prefix("assets/camera_fallback_image.gif").c_str(), 2.0f);

		std::array<std::array<Color, CAMERA_GRID_COLUMN_COUNT>, CAMERA_GRID_ROW_COUNT> colors = {{
			{Color{191, 0, 0, 255}, Color{0, 0, 191, 255}},//    LIME,   GOLD},
			{Color{0, 191, 0, 255}, Color{191, 191, 0, 255}},//  MAROON, VIOLET},
			// {ORANGE,   PINK, SKYBLUE,  BROWN}
		}};

		auto camera_grid = std::make_shared<dynamic_texture::HomogeneousDynamicTextureGrid2D<gui::CameraViewport, CAMERA_GRID_ROW_COUNT, CAMERA_GRID_COLUMN_COUNT>>(1112, 834);
		for(uint8_t y = 0; y < CAMERA_GRID_ROW_COUNT; y++){
			for(uint8_t x = 0; x < CAMERA_GRID_COLUMN_COUNT; x++){
				auto camera = std::make_unique<gui::CameraViewport>(
					1112, 834,
					fallback,
					colors[y][x],
					16
				);

				camera_grid->cells[y][x] = std::move(camera);
			}
		}

		return camera_grid;
	}

	std::shared_ptr<gui::JoystickDisplay> InitializeJoystickDisplay(){
		SetGamepadMappings(JOYSTICK_MAPPINGS);

		std::vector<Rectangle> joystick_display_button_indicators;
		{
			float overall_x_offset = 0.0f;
			float overall_y_offset = 0.0f;

			float button_size = 32.0f;
			float button_x_offset = 64.0f;
			float button_y_offset = 48.0f;

			int button_x_count = 12;
			int button_y_count = 1;

			for(int y = 0; y < button_y_count; y++){
				for(int x = 0; x < button_x_count; x++){
					joystick_display_button_indicators.push_back(Rectangle{overall_x_offset + (x * button_x_offset), overall_y_offset + (y * button_y_offset), button_size, button_size});
				}
			}
		}
		std::vector<Rectangle> joystick_display_axis_indicators;
		{
			float overall_x_offset = 0.0f;
			float overall_y_offset = 64.0f;

			float axis_width = 288.0f;
			float axis_height = 32.0f;
			float axis_x_offset = 384.0f;
			float axis_y_offset = 48.0f;

			int axis_x_count = 2;
			int axis_y_count = 2;

			for(int y = 0; y < axis_y_count; y++){
				for(int x = 0; x < axis_x_count; x++){
					joystick_display_axis_indicators.push_back(Rectangle{overall_x_offset + (x * axis_x_offset), overall_y_offset + (y * axis_y_offset), axis_width, axis_height});
				}
			}
		}
		std::vector<const char*> joystick_display_axis_labels;
		{
			joystick_display_axis_labels.push_back(" Roll: %.02f");
			joystick_display_axis_labels.push_back(" Pitch: %.02f");
			joystick_display_axis_labels.push_back(" Yaw: %.02f");
			joystick_display_axis_labels.push_back(" Throttle: %.02f");
		}

		return std::make_shared<gui::JoystickDisplay>(0, joystick_display_button_indicators, joystick_display_axis_indicators, joystick_display_axis_labels);
	}

	std::shared_ptr<gui::OrientationViewport> InitializeOrientationViewport(){
		return std::make_shared<gui::OrientationViewport>(horrible_ros_path::prepend_prefix("assets/rov.obj").c_str(), 608, 400);
	}

	std::shared_ptr<gui::ArtificialHorizon> InitializeArtificialHorizon(){
		return std::make_shared<gui::ArtificialHorizon>(200);
	}

	std::shared_ptr<gui::SshDisplay> InitializeSshDisplay(){
		return std::make_shared<gui::SshDisplay>(
			8, 15, 80, 24, 100,
			LoadFontEx(horrible_ros_path::prepend_prefix("assets/fonts/font.ttf").c_str(), 60, nullptr, 256),
			LoadFontEx(horrible_ros_path::prepend_prefix("assets/fonts/font_bold.ttf").c_str(), 60, nullptr, 256)
		);
	}
}

#endif
