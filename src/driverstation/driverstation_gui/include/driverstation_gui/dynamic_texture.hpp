#ifndef DYNAMIC_TEXTURE_HEADER_INCLUDED
#define DYNAMIC_TEXTURE_HEADER_INCLUDED

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <raylib.h>
#include <tuple>

#include "modes_util.hpp"
#include "ui_util.hpp"

namespace driverstation::dynamic_texture{
	struct DynamicTexture2D{
		virtual void unload() = 0;
		virtual Texture2D current_texture() = 0;
		virtual void on_click(float x, float y) = 0;
		virtual std::tuple<uint, uint> size() = 0;

		Rectangle rectangle(float x = 0.0f, float y = 0.0f){
			auto [width, height] = this->size();
			return Rectangle{x, y, (float)width, (float)height};
		};

		// TODO: Necessary?
		void drawRec(Rectangle rectangle){
			DrawTexturePro(
				this->current_texture(),
				this->rectangle(),
				rectangle,
				Vector2{0.0f, 0.0f}, 0.0f, WHITE
			);
		}

		void draw(float x, float y, float width, float height){
			this->drawRec(Rectangle{x, y, width, height});
		}

		void draw(float x, float y){
			auto [width, height] = this->size();
			this->draw(x, y, (float)width, (float)height);
		}
	};

	struct DynamicRenderTexture2D : public DynamicTexture2D{
		RenderTexture2D render_texture;

		DynamicRenderTexture2D(uint width, uint height){
			this->render_texture = LoadRenderTexture(width, height);
		}

		// TODO: Are these unloads necessary if raylib handles all unloading when the program exits?
		void unload() override{
			UnloadRenderTexture(this->render_texture);
		}

		Texture2D current_texture() override{
			return this->render_texture.texture;
		}

		void on_click([[maybe_unused]] float x, [[maybe_unused]] float y) override{}

		std::tuple<uint, uint> size() override{
			return std::make_tuple(
				this->render_texture.texture.width,
				this->render_texture.texture.height
			);
		}
	};

	// TODO: Is the requirement of the same type for each cell necessary? It's only there to enforce the camera grid's types
	template<typename T, uint8_t row_count, uint8_t column_count, typename std::enable_if<std::is_base_of_v<DynamicTexture2D, T>>::type* = nullptr>
	struct HomogeneousDynamicTextureGrid2D : public DynamicRenderTexture2D{
		std::array<std::array<std::unique_ptr<T>, column_count>, row_count> cells;

		HomogeneousDynamicTextureGrid2D(int width, int height) : DynamicRenderTexture2D(width, height){
			this->cell_view_size = Vector2{(float)width / column_count, (float)height / row_count};
		}

		// TODO: Better way of initializing cells? Builder? Template?
		void setCell(uint8_t row, uint8_t column, std::unique_ptr<T> cell){
			this->cells[row][column] = cell;
		}

		Texture2D current_texture() override{
			auto [y, x] = (std::tuple<uint8_t, uint8_t>)this->selected_cell.value_or(std::make_tuple(0, 0));
			{const modes::TextureMode main_texture(this->render_texture);
				if(this->selected_cell && this->cells[y][x]){
					this->cells[y][x]->drawRec(this->rectangle());
				}else{
					for(uint8_t y = 0; y < row_count; y++){
						for(uint8_t x = 0; x < column_count; x++){
							if(this->cells[y][x]){
								this->cells[y][x]->draw(
									x * this->cell_view_size.x, y * this->cell_view_size.y,
									this->cell_view_size.x, this->cell_view_size.y
								);
							}
						}
					}
				}
			}

			util::rendering::FlipVertically(this->render_texture);

			return DynamicRenderTexture2D::current_texture();
		}

		// TODO: Should the body of this method be integrated into current_texture()?
		std::optional<std::tuple<uint8_t, uint8_t>> update([[maybe_unused]] int joystick_id = -1){
			// Mouse Control:
			//   Click on an unfocused cell to bring it in to focus
			//   Click on a focused cell to bring it out of focus
			if(this->selected_cell){
				// TODO: This isn't accounting for being offset from the origin
				if(IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(GetMousePosition(), this->rectangle())) this->deselectCell();
			}else{
				if(IsMouseButtonPressed(MOUSE_BUTTON_LEFT)){
					auto [width, height] = this->size();
					// TODO: This doesn't have a concept of its own position (it only works because it starts at the origin)
					Vector2 mouse_position_within_rectangle = GetMousePosition();

					if(
						mouse_position_within_rectangle.x >= 0.0f && mouse_position_within_rectangle.y >= 0.0f &&
						mouse_position_within_rectangle.x < width && mouse_position_within_rectangle.y < height
					){
						this->setSelectedCell(
							mouse_position_within_rectangle.y / this->cellViewSize().y,
							mouse_position_within_rectangle.x / this->cellViewSize().x
						);
					}
				}
			}

			return this->selectedCell();
		}

		// TODO: Should this fall back to no selected cell on fail?
		bool setSelectedCell(uint8_t row, uint8_t column){
			if(row < row_count && column < column_count){
				this->selected_cell = std::make_tuple(row, column);
				return true;
			}
			return false;
		}

		// TODO: Should this fall back to no selected cell on fail?
		bool setSelectedCellIndex(uint8_t index){
			if(index < row_count * column_count){
				this->selected_cell = std::make_tuple(index / column_count, index % column_count);
				return true;
			}
			return false;
		}

		// TODO: Is this necessary? Can this be integrated into another setter function?
		void deselectCell(){
			this->selected_cell = std::nullopt;
		}

		std::optional<std::tuple<uint8_t, uint8_t>> selectedCell(){return this->selected_cell;}

		constexpr std::tuple<uint8_t, uint8_t> gridSize(){return std::make_tuple(column_count, row_count);}
		constexpr Vector2 cellViewSize(){return this->cell_view_size;}

		private:
			std::optional<std::tuple<uint8_t, uint8_t>> selected_cell = std::nullopt;
			Vector2 cell_view_size;
	};

	template<uint8_t row_count, uint8_t column_count>
	using HeterogeneousDynamicTextureGrid2D = HomogeneousDynamicTextureGrid2D<DynamicTexture2D, row_count, column_count>;

	struct GifTexture2D : public DynamicTexture2D{
		Image frame_data;
		int frame_count;
		double frame_rate;
		Texture2D texture;

		GifTexture2D(const char* file_name, double frame_rate){
			this->frame_rate = frame_rate;
			this->frame_data = LoadImageAnim(file_name, &this->frame_count);
			this->texture = LoadTextureFromImage(this->frame_data);
		} // derive_unique(GifTexture2D); derive_shared(GifTexture2D);  // TODO: Remove?

		void unload() override{
			UnloadTexture(this->texture);
			UnloadImage(this->frame_data);
		}

		Texture2D current_texture() override{
			unsigned int expected_frame_number = (unsigned int)(GetTime() * this->frame_rate) % this->frame_count;
			// Multiply by 4 for the 4 channels in an RGBA image
			unsigned int frame_data_offset = this->frame_data.width * this->frame_data.height * 4 * expected_frame_number;
			UpdateTexture(this->texture, (unsigned char*)this->frame_data.data + frame_data_offset);
			return this->texture;
		}

		void on_click([[maybe_unused]] float x, [[maybe_unused]] float y) override{}

		std::tuple<uint, uint> size() override{
			return std::make_tuple(
				this->texture.width,
				this->texture.height
			);
		}
	};

	struct ColorBox : public DynamicRenderTexture2D{
		Color color;

		ColorBox(Color color, uint width, uint height) :
			DynamicRenderTexture2D(width, height),
			color(color){}

		Texture2D current_texture() override{
			{const modes::TextureMode main_texture(this->render_texture);
				DrawRectangleRec(this->rectangle(), this->color);
			}

			return DynamicRenderTexture2D::current_texture();
		}
	};

	namespace{
		struct MultiCellDynamicTextureGrid2DCell{
			std::shared_ptr<DynamicTexture2D> value;
			uint8_t cell_count_x;
			uint8_t cell_count_y;
		};
	}

	template<uint8_t row_count, uint8_t column_count>
	struct MultiCellDynamicTextureGrid2DSelectFrom;

	template<uint8_t row_count, uint8_t column_count>
	struct MultiCellDynamicTextureGrid2DSelectFromTo;

	template<uint8_t row_count, uint8_t column_count>
	struct MultiCellDynamicTextureGrid2D : public DynamicRenderTexture2D{
		friend struct MultiCellDynamicTextureGrid2DSelectFromTo<row_count, column_count>;

		Color background_color;

		MultiCellDynamicTextureGrid2D(Color background_color, uint width, uint height) :
			DynamicRenderTexture2D(width, height),
			background_color(background_color)
		{
			this->cell_view_size = Vector2{(float)width / column_count, (float)height / row_count};
		}

		std::optional<Rectangle> bounds_of(std::shared_ptr<DynamicTexture2D>&& cell_element){
			for(uint8_t y = 0; y < row_count; y++){
				for(uint8_t x = 0; x < column_count; x++){
					MultiCellDynamicTextureGrid2DCell* cell = &this->cells[y][x];
					if(cell->value == cell_element){
						return Rectangle{
							x * this->cell_view_size.x,
							y * this->cell_view_size.y,
							cell->cell_count_x * this->cell_view_size.x,
							cell->cell_count_y * this->cell_view_size.y
						};
					}
				}
			}

			return std::nullopt;
		}

		Texture2D current_texture() override{
			{const modes::TextureMode main_texture(this->render_texture);
				ClearBackground(background_color);

				for(uint8_t y = 0; y < row_count; y++){
					for(uint8_t x = 0; x < column_count; x++){
						MultiCellDynamicTextureGrid2DCell* cell = &this->cells[y][x];
						if(cell->value){
							cell->value->draw(
								x * this->cell_view_size.x,
								y * this->cell_view_size.y,
								cell->cell_count_x * this->cell_view_size.x,
								cell->cell_count_y * this->cell_view_size.y
							);
						}
					}
				}
			}

			util::rendering::FlipVertically(this->render_texture);

			return DynamicRenderTexture2D::current_texture();
		}

		void on_click(float x, float y) override{
			float x_in_cells = x * row_count;
			float y_in_cells = y * column_count;

			uint8_t inter_cell_x = std::floor(x_in_cells);
			uint8_t inter_cell_y = std::floor(y_in_cells);

			float intra_cell_x = fmod(x_in_cells, 1.0f);
			float intra_cell_y = fmod(y_in_cells, 1.0f);

			for(uint8_t y = 0; y <= inter_cell_y; y++){
				for(uint8_t x = 0; x <= inter_cell_x; x++){
					MultiCellDynamicTextureGrid2DCell* cell = &this->cells[y][x];
					if(cell->value){
						uint8_t end_x = x + cell->cell_count_x - 1;
						uint8_t end_y = y + cell->cell_count_y - 1;

						if(end_x >= inter_cell_x && end_y >= inter_cell_y){
							float x_in_component = ((inter_cell_x - x) + intra_cell_x) / (cell->cell_count_x);
							float y_in_component = ((inter_cell_y - y) + intra_cell_y) / (cell->cell_count_y);

							cell->value->on_click(x_in_component, y_in_component);
						}
					}
				}
			}
		}

		MultiCellDynamicTextureGrid2DSelectFrom<row_count, column_count> from(uint8_t x, uint8_t y){
			return MultiCellDynamicTextureGrid2DSelectFrom(this, x, y);
		}

		private:
			std::array<std::array<MultiCellDynamicTextureGrid2DCell, column_count>, row_count> cells;
			Vector2 cell_view_size;
	};

	template<uint8_t row_count, uint8_t column_count>
	struct MultiCellDynamicTextureGrid2DSelectFrom{
		friend struct MultiCellDynamicTextureGrid2D<row_count, column_count>;

		MultiCellDynamicTextureGrid2DSelectFromTo<row_count, column_count> to(uint8_t x, uint8_t y){
			return MultiCellDynamicTextureGrid2DSelectFromTo(
				this->multi_cell,
				this->from_x, this->from_y,
				x, y
			);
		}

		private:
			MultiCellDynamicTextureGrid2D<row_count, column_count>* multi_cell;
			uint8_t from_x;
			uint8_t from_y;

			MultiCellDynamicTextureGrid2DSelectFrom(
				MultiCellDynamicTextureGrid2D<row_count, column_count>* multi_cell,
				uint8_t from_x, uint8_t from_y
			): multi_cell(multi_cell), from_x(from_x), from_y(from_y){}
	};

	template<uint8_t row_count, uint8_t column_count>
	struct MultiCellDynamicTextureGrid2DSelectFromTo{
		friend struct MultiCellDynamicTextureGrid2DSelectFrom<row_count, column_count>;

		MultiCellDynamicTextureGrid2D<row_count, column_count>* set(std::shared_ptr<DynamicTexture2D>&& value){
			this->multi_cell->cells[this->from_y][this->from_x] = MultiCellDynamicTextureGrid2DCell{
				std::move(value), this->to_x - this->from_x, this->to_y - this->from_y
			};

			return this->multi_cell;
		}

		MultiCellDynamicTextureGrid2DSelectFromTo tapSizeCells(uint8_t* width, uint8_t* height){
			*width = this->to_x - this->from_x;
			*height = this->to_y - this->from_y;

			return *this;
		}

		MultiCellDynamicTextureGrid2DSelectFromTo tapSizePixels(uint32_t* width, uint32_t* height){
			*width = (this->to_x - this->from_x) * this->multi_cell->cell_view_size.x;
			*height = (this->to_y - this->from_y) * this->multi_cell->cell_view_size.y;

			return *this;
		}

		private:
			MultiCellDynamicTextureGrid2D<row_count, column_count>* multi_cell;
			uint8_t from_x;
			uint8_t from_y;
			uint8_t to_x;
			uint8_t to_y;

			MultiCellDynamicTextureGrid2DSelectFromTo(
				MultiCellDynamicTextureGrid2D<row_count, column_count>* multi_cell,
				uint8_t from_x, uint8_t from_y, uint8_t to_x, uint8_t to_y
			): multi_cell(multi_cell), from_x(from_x), from_y(from_y), to_x(to_x), to_y(to_y){}
	};
}

#endif