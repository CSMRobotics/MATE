#ifndef CAMERA_VIEWPORT_UI_HEADER_INCLUDED
#define CAMERA_VIEWPORT_UI_HEADER_INCLUDED

#include <raylib.h>

#include "dynamic_texture.hpp"
#include "modes_util.hpp"
#include "ui_util.hpp"

namespace driverstation::gui{
	struct CameraViewport : public dynamic_texture::DynamicRenderTexture2D{
		std::shared_ptr<dynamic_texture::DynamicTexture2D> fallback_image;
		Color border_color;
		int border_width;

		CameraViewport(
			uint width, uint height, std::shared_ptr<dynamic_texture::DynamicTexture2D> fallback_image,
			Color border_color = ColorAlpha(BLUE, 0.75f), int border_width = 8
		) : dynamic_texture::DynamicRenderTexture2D(width, height),
			fallback_image(fallback_image),
			border_color(border_color),
			border_width(border_width){}

		void unload() override{
			this->fallback_image->unload();
			dynamic_texture::DynamicRenderTexture2D::unload();
		}

		Texture2D current_texture() override{
			auto [width, height] = this->size();
			Rectangle destination_rectangle = Rectangle{0.0f, 0.0f, (float)width, (float)height};
			{const modes::TextureMode main_texture(this->render_texture);
				this->fallback_image->drawRec(destination_rectangle);
				// TODO: Transparency applies to the render texture itself, not to what is being drawn
				DrawRectangleLinesEx(destination_rectangle, this->border_width, this->border_color);
			}
			// TODO: There was some issue with flipping the texture with a border on it; so for now, just draw the border after (it's symmetric, after all)
			util::rendering::FlipVertically(this->render_texture);

			return dynamic_texture::DynamicRenderTexture2D::current_texture();
		}
	};
}

#endif
