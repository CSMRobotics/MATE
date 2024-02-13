#ifndef ESTOP_UI_HEADER_INCLUDED
#define ESTOP_UI_HEADER_INCLUDED

#include <functional>
#include <optional>
#include <raylib.h>

#include "modes_util.hpp"
#include "ui_util.hpp"

namespace driverstation::gui{
	struct EStop : public dynamic_texture::DynamicRenderTexture2D{
		EStop(uint width, uint height, uint8_t molly_guard_level, std::optional<std::function<void(void)>> trigger_callback = std::nullopt) : dynamic_texture::DynamicRenderTexture2D(width, height),
			mollyguard_level(molly_guard_level),trigger_callback(trigger_callback){
			this->recalculate_colors();
		}

		Texture2D current_texture() override{
			auto [width, height] = this->size();
			{const modes::TextureMode main_texture(this->render_texture);
				ClearBackground(this->current_base_color);
				DrawCircle(width >> 1, height >> 1, 0.325f * std::min(width, height), this->current_button_color);

				// Rectangle this_rectangle = this->rectangle();
				// for(uint8_t _ = 0; _ < mollyguard_level; _++){
				// 	DrawRectangleRec(this_rectangle, ColorAlpha(LIGHTGRAY, .25f));
				// }
			}
			
			return dynamic_texture::DynamicRenderTexture2D::current_texture();
		}

		void on_click([[maybe_unused]] float x, [[maybe_unused]] float y) override{
			if(mollyguard_level > 0){
				mollyguard_level--;
				this->recalculate_colors();
			}else if(this->trigger_callback.has_value()){
				(*this->trigger_callback)();
			}
		}

		std::optional<std::function<void(void)>> setTriggerCallback(std::function<void(void)> callback){
			if(this->trigger_callback.has_value()){
				std::function<void(void)> previous_callback = this->trigger_callback.value();
				this->trigger_callback = callback;
				return previous_callback;
			}else{
				this->trigger_callback = callback;
				return std::nullopt;
			}
		}

		private:
			uint8_t mollyguard_level;
			Color base_color = YELLOW;
			Color button_color = RED;
			Color current_base_color;
			Color current_button_color;
			std::optional<std::function<void(void)>> trigger_callback;

			void recalculate_colors(){
				float blend_level = 1.0f - std::pow(0.5f, mollyguard_level);
				Color overlay_color = ColorAlpha(GRAY, blend_level);
				this->current_base_color = ColorAlphaBlend(this->base_color, overlay_color, WHITE);
				this->current_button_color = ColorAlphaBlend(this->button_color, overlay_color, WHITE);
			}
	};
}

#endif
