#ifndef JOYSTICK_UI_HEADER_INCLUDED
#define JOYSTICK_UI_HEADER_INCLUDED

#include <assert.h>
#include <memory>
#include <raylib.h>
#undef RAYGUI_IMPLEMENTATION
#include <raygui.h>
#include <vector>

#include "vec_rec_util.hpp"

namespace driverstation::gui{
	struct JoystickDisplay{
		int joystick_id;
		std::vector<Rectangle> button_indicators;
		std::vector<Rectangle> axis_indicators;
		std::vector<const char*> axis_labels;

		// TODO: Should these kinds of things take floats as arguments instead?
		JoystickDisplay(int joystick_id, std::vector<Rectangle> button_indicators, std::vector<Rectangle> axis_indicators, std::vector<const char*> axis_labels){
			assert(axis_indicators.size() == axis_labels.size());

			this->joystick_id = joystick_id;
			this->button_indicators = button_indicators;
			this->axis_indicators = axis_indicators;
			this->axis_labels = axis_labels;
		}

		// TODO: Should JoystickDisplay use a RenderTexture like other components?
		// TODO: parameter `color` is currently unused
		void draw(int x, int y, [[maybe_unused]] Color color){
			bool joystick_available = IsGamepadAvailable(this->joystick_id);

			int previous_gui_state = GuiGetState();
			if(joystick_available) GuiEnable();
			else GuiDisable();

			for(size_t i = 0; i < this->button_indicators.size(); i++){
				GuiCheckBox(
					this->button_indicators[i] + Vector2{(float)x, (float)y},
					TextFormat("%d", i + 1),
					IsGamepadButtonDown(this->joystick_id, i + 1)
				);
			}
			for(size_t i = 0; i < this->axis_indicators.size(); i++){
				// NOTE: Buttons are 1-indexed, Axes are 0-indexed
				float axis_value = GetGamepadAxisMovement(this->joystick_id, i);
				GuiProgressBar(
					this->axis_indicators[i] + Vector2{(float)x, (float)y},
					nullptr, TextFormat(this->axis_labels[i], axis_value),
					axis_value + 1.0f, 0.0f, 2.0f  // NOTE: Progress bars don't seem to like negative values
				);
			}

			GuiSetState(previous_gui_state);
		}
		void drawV(Vector2 position, Color color){this->draw((int)position.x, (int)position.y, color);}
	};
}

#endif
