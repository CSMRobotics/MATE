#ifndef ORINTATION_VIEWPORT_UI_HEADER_INCLUDED
#define ORINTATION_VIEWPORT_UI_HEADER_INCLUDED

#include <memory>
#include <raylib.h>

// TODO: Should be temporary
#include <math.h>

#include "modes_util.hpp"
#include "ui_util.hpp"

namespace driverstation::gui{
	struct OrientationViewport : public dynamic_texture::DynamicRenderTexture2D{
		Model model;
		bool wireframe = false;
		Vector3 rotation_axis = Vector3{0.0f, 1.0f, 0.0f};
		Vector3 scale;
		float rotation_angle = 0.0f;
		Camera3D camera = {
			Vector3{50.0f, 50.0f, 50.0f}, // position
			Vector3{0.0f, 10.0f, 0.0f},   // target
			// TODO: It looks like this can be changed to be +Z-up or other; matching whatever we end up using (but it doesn't like the default orbiting camera)
			Vector3{0.0f, 1.0f, 0.0f},    // up
			30.0f,                        // fovy
			CAMERA_PERSPECTIVE            // projection
		};

		// TODO: Add model texture properties (?)

		// TODO: Should these use a builder pattern?
		OrientationViewport(const char* model_file_name, int width, int height) : dynamic_texture::DynamicRenderTexture2D(width, height){
			this->model = LoadModel(model_file_name);
			this->scale = util::getNormalizingScale(this->model, 20.0f);

			// SetCameraMode(this->camera, CAMERA_CUSTOM);
			// NOTE: It looks like this should be called upon drawing if using multiple cameras
			SetCameraMode(this->camera, CAMERA_FREE);
		}

		// NOTE: It seems like this is better put in the `current_texture()` method
		// void update(){
		// 	UpdateCamera(&this->camera);
		// }

		void unload() override{
			UnloadModel(this->model);
			dynamic_texture::DynamicRenderTexture2D::unload();
		}

		Texture2D current_texture() override{
			// SetCameraMode(this->camera, CAMERA_ORBITAL);  // NOTE: It doesn't like this
			UpdateCamera(&this->camera);

			// When orbiting, only one axis position changes sign at a time; so if both change at once, flip back the signs
			// if(previous_camera_position.x * this->camera.position.x < 0.0f && previous_camera_position.z * this->camera.position.z < 0.0f){
			// 	this->camera.position.x = -this->camera.position.x;
			// 	this->camera.position.z = -this->camera.position.z;
			// }

			{const modes::TextureMode main_texture(this->render_texture);
				// TODO: Should background color be a property?
				ClearBackground(BLACK);
				{const modes::Mode3D model_view(this->camera);
					// TODO: Make a 'draw as wireframe' bool property?
					DrawModelEx(this->model, {0.0f, 0.0f, 0.0f}, this->rotation_axis, this->rotation_angle, this->scale, WHITE);

					DrawGrid(2, 10.0f);
					DrawLine3D({-10.0f,0.0f,-10.0f}, {10.0f,0.0f,-10.0f}, RED);
					DrawLine3D({-10.0f,0.0f,-10.0f}, {-10.0f,20.0f,-10.0f}, GREEN);
					DrawLine3D({-10.0f,0.0f,-10.0f}, {-10.0f,0.0f,10.0f}, BLUE);
				}
			}
			util::rendering::FlipVertically(this->render_texture);

			return dynamic_texture::DynamicRenderTexture2D::current_texture();
		}
	};
}

#endif
