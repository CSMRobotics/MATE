#ifndef ARTIFICIAL_HORIZON_UI_HEADER_INCLUDED
#define ARTIFICIAL_HORIZON_UI_HEADER_INCLUDED

#include <raylib.h>

// TODO: Should be temporary
#include <math.h>

#include "dynamic_texture.hpp"
#include "modes_util.hpp"
#include "ui_util.hpp"

#include "horrible_ros_path.hpp"

namespace driverstation::gui{
	struct ArtificialHorizon : public dynamic_texture::DynamicRenderTexture2D{
		Vector3 gravity_vector = Vector3{0.0f, 1.0f, 0.0f};
		Camera3D camera = {
			Vector3{0.0f, 0.0f, 100.0f}, // position
			Vector3{0.0f, 0.0f, 0.0f},   // target
			Vector3{0.0f, 1.0f, 0.0f},   // up
			30.0f,                       // fovy
			CAMERA_ORTHOGRAPHIC          // projection
		};
		Model hemisphere_model = LoadModelFromMesh(GenMeshHemiSphere(10.0f, 12.0f, 12.0f));
		Model sphere_model = LoadModel(horrible_ros_path::prepend_prefix("assets/navball.obj").c_str());
		Texture2D texture = LoadTexture(horrible_ros_path::prepend_prefix("assets/textures/navball.png").c_str());

		// TODO: Properties for colors?

		// TODO: Should these use a builder pattern?
		ArtificialHorizon(int size) : dynamic_texture::DynamicRenderTexture2D(size, size){
			// NOTE: It looks like this should be called upon drawing if using multiple cameras
			// SetCameraMode(this->camera, CAMERA_CUSTOM);  // Should not be movable
			// SetCameraMode(this->camera, CAMERA_ORBITAL);
			this->sphere_model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = this->texture;
		}

		// TODO: Should these be uint32_t?
		ArtificialHorizon(int width, int height) : dynamic_texture::DynamicRenderTexture2D(width, height){
			// NOTE: It looks like this should be called upon drawing if using multiple cameras
			// SetCameraMode(this->camera, CAMERA_CUSTOM);  // Should not be movable
			// SetCameraMode(this->camera, CAMERA_ORBITAL);
			this->sphere_model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = this->texture;
		}

		// NOTE: It seems like this is better put in the `current_texture()` method
		// void update(){
		// 	UpdateCamera(&this->camera);
		// }

		void unload() override{
			UnloadModel(this->hemisphere_model);
			dynamic_texture::DynamicRenderTexture2D::unload();
		}

		Texture2D current_texture() override{
			// SetCameraMode(this->camera, CAMERA_CUSTOM);  // Should not be movable
			// UpdateCamera(&this->camera);  // Don't update as it shouldn't move
			{const modes::TextureMode main_texture(this->render_texture);
				// TODO: Should background color be a property?
				ClearBackground(BLACK);  // Get current style background color?
				{const modes::Mode3D artificial_horizon(this->camera);
					DrawModelEx(
						this->sphere_model,
						{0.0f, 0.0f, 0.0f},
						{0.0f, 0.0f, 0.0f},
						0.0f,
						{16.0f, 16.0f, 16.0f},
						WHITE
					);
				}
			}
			util::rendering::FlipVertically(this->render_texture);

			return dynamic_texture::DynamicRenderTexture2D::current_texture();
		}
	};
}

#endif
