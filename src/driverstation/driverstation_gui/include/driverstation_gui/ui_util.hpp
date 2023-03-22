#ifndef UI_UTIL_HEADER_INCLUDED
#define UI_UTIL_HEADER_INCLUDED

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <raylib.h>
#include <tuple>

#include "modes_util.hpp"

namespace driverstation::util{
	Rectangle normalizeSigns(Rectangle rectangle){
		bool flip_x = (rectangle.width < 0.0f);
		bool flip_y = (rectangle.height < 0.0f);

		return Rectangle{
			(flip_x) ? rectangle.x + rectangle.width : rectangle.x,
			(flip_y) ? rectangle.y + rectangle.height : rectangle.y,
			(flip_x) ? -rectangle.width : rectangle.width,
			(flip_y) ? -rectangle.height : rectangle.height
		};
	}

	Vector3 getNormalizingScale(Model model, float maximum_dimension = 1.0f){
		BoundingBox bounding_box = GetModelBoundingBox(model);
		float model_maximum_dimension = std::max(std::max(
		    bounding_box.max.x - bounding_box.min.x,
		    bounding_box.max.y - bounding_box.min.y
		),  bounding_box.max.z - bounding_box.min.z
		);

		float scale_value = maximum_dimension / model_maximum_dimension;

		return Vector3{scale_value, scale_value, scale_value};
	}

	namespace measurement{
		namespace{
			Vector2 last_mouse_down_position;
		}

		void HandleRectVisualization(){
			if(IsKeyDown(KEY_LEFT_CONTROL)){
				if(IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) last_mouse_down_position = GetMousePosition();
				if(IsMouseButtonDown(MOUSE_LEFT_BUTTON)){
					Vector2 current_mouse_position = GetMousePosition();
					Rectangle bounding_box = normalizeSigns(Rectangle{
						last_mouse_down_position.x,
						last_mouse_down_position.y,
						current_mouse_position.x - last_mouse_down_position.x,
						current_mouse_position.y - last_mouse_down_position.y
					});
					DrawRectangleRec(bounding_box, ColorAlpha(ORANGE, 0.25f));
					DrawRectangleLinesEx(bounding_box, 1.0f, ORANGE);
					const char* info_text = TextFormat(
						"%d, %d, %d, %d",
						(int)bounding_box.x, (int)bounding_box.y,
						(int)bounding_box.width, (int)bounding_box.height
					);
					DrawText(info_text, 20, 20, 20, ORANGE);

					if(IsKeyPressed(KEY_C)) SetClipboardText(TextFormat("Rectangle{%s}", info_text));
				}else{
					const char* info_text = TextFormat(
						"%d, %d",
						GetMouseX(), GetMouseY()
					);
					DrawText(info_text, 20, 20, 20, ORANGE);

					if(IsKeyPressed(KEY_C)) SetClipboardText(TextFormat("Vector2{%s}", info_text));
				}
			}
		}
	}

	namespace rendering{
		namespace{
			RenderTexture2D buffer;
		}

		void InitRendering(){
			buffer = LoadRenderTexture(2048, 2048);
		}

		void FlipHorizontally(RenderTexture2D render_texture){
			{const modes::TextureMode clear_buffer(buffer);
				ClearBackground(BLACK);
				DrawTexture(render_texture.texture, 0, 0, WHITE);
			}

			Vector2 render_texture_size = Vector2{(float)render_texture.texture.width,  (float)render_texture.texture.height};
			{const modes::TextureMode main_texture(render_texture);
				DrawTexturePro(
					buffer.texture,
					Rectangle{0.0f, (float)buffer.texture.height - render_texture_size.y, -render_texture_size.x, render_texture_size.y},
					Rectangle{0.0f, 0.0f, render_texture_size.x, render_texture_size.y},
					Vector2{0.0f, 0.0f}, 0.0f, WHITE
				);
			}
		}

		void FlipVertically(RenderTexture2D render_texture){
			{const modes::TextureMode clear_buffer(buffer);
				ClearBackground(BLACK);
				DrawTexture(render_texture.texture, 0, 0, WHITE);
			}

			Vector2 render_texture_size = Vector2{(float)render_texture.texture.width,  (float)render_texture.texture.height};
			{const modes::TextureMode main_texture(render_texture);
				DrawTexturePro(
					buffer.texture,
					Rectangle{0.0f, (float)buffer.texture.height - render_texture_size.y, render_texture_size.x, -render_texture_size.y},
					Rectangle{0.0f, 0.0f, render_texture_size.x, render_texture_size.y},
					Vector2{0.0f, 0.0f}, 0.0f, WHITE
				);
			}
		}

		void Shift(RenderTexture2D render_texture, int x, int y, Color backing_color = BLANK, Color tinting_color = WHITE){
			// TODO: Currently only works for non-negative values? (seems fine)
			{const modes::TextureMode clear_buffer(buffer);
				ClearBackground(backing_color);
				DrawTexture(render_texture.texture, 0, 0, WHITE);
			}

			Vector2 render_texture_size = Vector2{(float)render_texture.texture.width,  (float)render_texture.texture.height};
			{const modes::TextureMode main_texture(render_texture);
				ClearBackground(backing_color);
				DrawTexturePro(
					buffer.texture,
					Rectangle{0.0f, (float)buffer.texture.height - render_texture_size.y, render_texture_size.x, render_texture_size.y},
					Rectangle{(float)x, (float)y, render_texture_size.x, render_texture_size.y},
					Vector2{0.0f, 0.0f}, 0.0f, tinting_color);
			}
		}

		void UnloadRendering(){
			UnloadRenderTexture(buffer);
		}
	}
}

#endif
