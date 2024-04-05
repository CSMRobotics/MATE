#ifndef MODES_UTIL_HEADER_INCLUDED
#define MODES_UTIL_HEADER_INCLUDED

#include <raylib.h>
#include <vector>

namespace driverstation::modes{
	namespace{
		std::vector<RenderTexture2D> modes_stack;
	}

	class Drawing{
		public:
			Drawing(){BeginDrawing();}
			~Drawing(){EndDrawing();}
	};
	// class Mode2D{
	// 	public:
	// 		Mode2D(Camera2D camera){BeginMode2D(camera);}
	// 		~Mode2D(){EndMode2D();}
	// };
	class Mode3D{
		public:
			Mode3D(Camera3D camera){BeginMode3D(camera);}
			~Mode3D(){EndMode3D();}
	};
	class TextureMode{
		public:
			TextureMode(RenderTexture2D target){
				modes_stack.push_back(target);
				BeginTextureMode(target);
			}
			~TextureMode(){
				modes_stack.pop_back();

				if(modes_stack.size() == 0){
					EndTextureMode();
				}else{
					BeginTextureMode(modes_stack.back());
				}
			}
	};
	// class ShaderMode{
	// 	public:
	// 		ShaderMode(Shader shader){BeginShaderMode(shader);}
	// 		~ShaderMode(){EndShaderMode();}
	// };
	// class BlendMode{
	// 	public:
	// 		BlendMode(int mode){BeginBlendMode(mode);}
	// 		~BlendMode(){EndBlendMode();}
	// };
	// class ScissorMode{
	// 	public:
	// 		ScissorMode(int x, int y, int width, int height){BeginScissorMode(x, y, width, height);}
	// 		~ScissorMode(){EndScissorMode();}
	// };
	// class VrStereoMode{
	// 	public:
	// 		VrStereoMode(VrStereoConfig config){BeginVrStereoMode(config);}
	// 		~VrStereoMode(){EndVrStereoMode();}
	// };
}

#endif
