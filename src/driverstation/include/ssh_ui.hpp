#ifndef SSH_UI_HEADER_INCLUDED
#define SSH_UI_HEADER_INCLUDED

#include <cassert>
#include <iostream>
#include <memory>
#include <raylib.h>
#include <sstream>
#include <string>
#include <vector>

#include "modes_util.hpp"
#include "ssh_interface.hpp"
#include "ui_util.hpp"

// TODO: Should this be a property?
#define MAX_CHARACTERS_PER_TICK 2048

#define nonZeroOr(value, otherwise) ((value != 0) ? value : otherwise)

namespace driverstation::gui{
	struct SshDisplay : public dynamic_texture::DynamicRenderTexture2D{
		enum struct TextMode{
			DEFAULT,                                      // Normal text processing
			ESCAPE_CODE_BEGINNING,                        // Escape (0x1B) found
			CONTROL_SEQUENCE_INTRODUCER,                  // Parsing useful escape sequences
			OPERATING_SYSTEM_COMMAND,                     // Command for the OS, ignore
			ESCAPE_CODE_WITHIN_OPERATING_SYSTEM_COMMAND,  // Escape found in OSC, exit if string terminator
			AWAITING_STRING_TERMINATOR,                   // Inside unused String-Terminator-delimited types
			EXPECTING_STRING_TERMINATOR                   // Escape found inside unused String-Terminator-delimited types
		};

		enum TextEffects{
			// Bits:
			// 7 - Unused
			// 6 - Unused
			// 5 - Unused
			// 4 - Unused
			// 3 - Unused
			// 2 - Unused
			// 1 - Bright color
			// 0 - Bold font
			NONE = 0x00,
			BOLD = 0x01,
			BRIGHT = 0x02,

			INCREASED_INTENSITY = BOLD | BRIGHT
		};

		enum struct CsiParameterType{
			NUMBER,
			INTERMEDIATE,
			FINAL,
			OTHER
		};

		struct CsiParameter{
			CsiParameterType type;
			// NOTE: 16-bit integer because some codes can have integers beyond 255 (e.g. \e[?2004h)
			uint16_t value;
		};

		Rectangle history_rectangle;
		Vector2 character_spacing;
		RenderTexture2D history_render_texture;
		ssh_interface::SshInterface ssh_interface;

		// NOTE: These are in pixels, not cells
		Vector2 cursor_position = {0.0f, 0.0f};
		Vector2 max_cursor_position;
		float scroll_position = 0.0f;
		float max_scroll_position;

		TextMode current_text_mode = TextMode::DEFAULT;
		std::vector<CsiParameter> current_csi_parameters;

		Color default_foreground_color = WHITE;
		Color current_foreground_color = WHITE;
		Color default_background_color = Color{31, 31, 31, 255};
		Color current_background_color = Color{31, 31, 31, 255};

		uint8_t current_text_effects = TextEffects::NONE;

		Font font_normal;
		Font font_bold;

		// TODO: Properties for colors?

		SshDisplay(
			uint8_t character_x_spacing, uint8_t character_y_spacing,
			uint8_t column_count, uint8_t line_count, uint8_t history_length,
			Font font_normal, Font font_bold
		):
			dynamic_texture::DynamicRenderTexture2D(character_x_spacing * column_count, character_y_spacing * line_count),
			font_normal(font_normal),
			font_bold(font_bold)
		{
			auto [width, height] = (std::tuple<float, float>)this->size();

			// TODO: Clean up
			this->history_rectangle = Rectangle{
				0.0f, 0.0f,
				width, (float)(character_y_spacing * history_length)
			};

			this->character_spacing = Vector2{(float)character_x_spacing, (float)character_y_spacing};
			this->history_render_texture = LoadRenderTexture(this->history_rectangle.width, this->history_rectangle.height);
			this->max_cursor_position = Vector2{this->character_spacing.x * (column_count - 1.0f), this->character_spacing.y * (line_count - 1.0f)};
			this->max_scroll_position = this->history_rectangle.height - height;
			{const modes::TextureMode main_texture(this->history_render_texture);
				ClearBackground(this->current_background_color);
			}
		}

		int connectTo(const char* host, const char* username, const char* password){
			return this->ssh_interface.openChannel(host, username, password);
		}

		ssh_interface::SshInterface::ConnectionStage currentStage() const{return this->ssh_interface.currentStage();}

		void unload() override{
			// UnloadFont(this->font_normal);
			UnloadRenderTexture(this->history_render_texture);
			dynamic_texture::DynamicRenderTexture2D::unload();
		}

		// TODO: Integrate into `current_texture()`?
		void update(){
			std::optional<char> possible_character;
			char current_character;
			int characters_processed = 0;

			{const modes::TextureMode history_texture(this->history_render_texture);  // NOTE: Unused because of needing to exit texture mode when shifting
			// BeginTextureMode(this->history_render_texture);
				while(characters_processed++ < MAX_CHARACTERS_PER_TICK){
					this->ssh_interface >> possible_character;
					if(possible_character == std::nullopt) break;
					current_character = *possible_character;

					// TODO: Carrige returns
					if(this->cursor_position.x > this->max_cursor_position.x || current_character == '\n'){
						this->cursor_position.x = 0.0f;
						// EndTextureMode();
						this->moveCursorY(1);
						// BeginTextureMode(this->history_render_texture);
					}
					switch(this->current_text_mode){
						case TextMode::DEFAULT:
						default:
							switch(current_character){
								case '\b':  // Backspace
									this->moveCursorX(-1);
									break;
								case 0x1B:  // Escape code
									this->current_text_mode = TextMode::ESCAPE_CODE_BEGINNING;
									break;
								default:
									if(current_character < 0x20) break;  // Other control character
									DrawRectangleV(
										this->cursor_position,
										this->character_spacing,
										this->current_background_color
									);

									DrawTextEx(
										(this->current_text_effects & BOLD) ? this->font_bold : this->font_normal,
										TextFormat("%c", current_character),
										Vector2{this->cursor_position.x + 1.0f, this->scroll_position + this->cursor_position.y},
										this->character_spacing.y, 2, this->current_foreground_color
									);
									// DrawText(TextFormat("%c", current_character), this->cursor_position.x + 1.0f, this->scroll_position + this->cursor_position.y, 16, this->current_foreground_color);
									// this->moveCursorX(1, true, true);
									this->cursor_position.x += this->character_spacing.x;
									break;
							}
							break;
						case TextMode::ESCAPE_CODE_BEGINNING:
							switch(current_character){
								case 'N':  // Single Shift Two
								case 'O':  // Single Shift Three
								case '\\':  // String Terminator
									this->current_text_mode = TextMode::DEFAULT;
									break;
								case '[':  // Control Sequence Introducer
									this->current_text_mode = TextMode::CONTROL_SEQUENCE_INTRODUCER;
									break;
								case ']':  // Operating System Command
									this->current_text_mode = TextMode::OPERATING_SYSTEM_COMMAND;
									break;
								case 'P':  // Device Control String
								case 'X':  // Start of String
								case '^':  // Privacy Message
								case '_':  // Application Program Command
									this->current_text_mode = TextMode::AWAITING_STRING_TERMINATOR;
									break;
							}
							break;
						case TextMode::CONTROL_SEQUENCE_INTRODUCER:
							if(current_character >= 0x30 && current_character <= 0x3F){        // Parameter byte, [0-9:;<=>?]
								if(current_character >= '0' && current_character <= '9'){
									if(this->current_csi_parameters.size() == 0 || this->current_csi_parameters.back().type != CsiParameterType::NUMBER){
										this->current_csi_parameters.push_back(CsiParameter{
											CsiParameterType::NUMBER,
											(uint16_t)(current_character - '0')
										});
									}else{  // NOTE: Preexisting integer literal
										this->current_csi_parameters.back().value *= 10;
										this->current_csi_parameters.back().value += (uint16_t)(current_character - '0');
									}
								}else{
									this->current_csi_parameters.push_back(CsiParameter{
										CsiParameterType::OTHER,
										(uint16_t)current_character
									});
								}
							}else if(current_character >= 0x20 && current_character <= 0x2F){  // Intermediate byte, [!"#$%&'()*+,-./]
								this->current_csi_parameters.push_back(CsiParameter{
									CsiParameterType::INTERMEDIATE,
									(uint16_t)current_character
								});
							}else if(current_character >= 0x40 && current_character <= 0x7E){  // Final byte, [@A–Z[\]^_`a–z{|}~], exit to TextMode::default text mode
								if(current_character == '['){  // TODO: htop will send things like '\x1B[27[3;35H', which might be intended to be one code
									this->current_csi_parameters.push_back(CsiParameter{
										CsiParameterType::OTHER,
										(uint16_t)current_character
									});
									break;
								}
								this->applyCsiCode(this->current_csi_parameters, current_character);
								// TODO: Is there a better way to do this?
								while(this->current_csi_parameters.size() > 0) this->current_csi_parameters.pop_back();
								this->current_text_mode = TextMode::DEFAULT;
							}
							break;
						case TextMode::OPERATING_SYSTEM_COMMAND:
							switch(current_character){
								case '\a':  // For historical reasons, xterm can end OSCs with a bell
									this->current_text_mode = TextMode::DEFAULT;
									break;
								case '0x1B':  // Escape, possibly String Terminator
									this->current_text_mode = TextMode::ESCAPE_CODE_WITHIN_OPERATING_SYSTEM_COMMAND;
									break;
							}
							break;
						case TextMode::ESCAPE_CODE_WITHIN_OPERATING_SYSTEM_COMMAND:
							switch(current_character){
								case '\\':  // String terminator code, exit to TextMode::default text mode
									this->current_text_mode = TextMode::DEFAULT;
									break;
								default:
									this->current_text_mode = TextMode::OPERATING_SYSTEM_COMMAND;
									break;
							}
							break;
						case TextMode::AWAITING_STRING_TERMINATOR:
							switch(current_character){
								case '0x1B':  // Escape, possibly String Terminator
									this->current_text_mode = TextMode::EXPECTING_STRING_TERMINATOR;
									break;
							}
							break;
						case TextMode::EXPECTING_STRING_TERMINATOR:
							switch(current_character){
								case '\\':  // String terminator code, exit to TextMode::default text mode
									this->current_text_mode = TextMode::DEFAULT;
									break;
								default:
									this->current_text_mode = TextMode::AWAITING_STRING_TERMINATOR;
									break;
							}
							break;
					}
				}
			// EndTextureMode();
			}
		}

		Texture2D current_texture() override{
			this->update();
			// TODO: Replace
			int cursor_y_position = this->history_rectangle.height - this->character_spacing.y;
			{const modes::TextureMode TODO_REPLACE_1(this->history_render_texture);
				if(GetTime() - (int)GetTime() < 0.5f) DrawRectangleV({this->cursor_position.x,this->scroll_position+this->cursor_position.y+12},{this->character_spacing.x,2},WHITE);
			}

			{const modes::TextureMode main_texture(this->render_texture);
				// NOTE: Subtraction due to the vertical flip of render textures
				Rectangle clipping_rectangle = this->rectangle(0.0f, this->max_scroll_position - this->scroll_position);

				DrawTexturePro(
					this->history_render_texture.texture,
					clipping_rectangle, this->rectangle(),
					Vector2{0.0f, 0.0f}, 0.0f, WHITE
				);
			}
			// NOTE: Appears unnecessary
			// util::rendering::FlipVertically(this->render_texture);

			// TODO: Replace
			{const modes::TextureMode TODO_REPLACE_2(this->history_render_texture);
				DrawRectangleV(Vector2{this->cursor_position.x,this->scroll_position+this->cursor_position.y+12},{this->character_spacing.x,2},this->current_background_color);
			}

			return dynamic_texture::DynamicRenderTexture2D::current_texture();
		}

		void moveCursorX(float delta, bool in_columns = true, bool wrap_at_end = false){
			if(in_columns) delta *= this->character_spacing.x;
			this->cursor_position.x += delta;
			if(this->cursor_position.x < 0.0f){
				this->cursor_position.x = 0.0f;
			}else if(this->cursor_position.x > this->max_cursor_position.x){
				if(wrap_at_end){
					this->cursor_position.x = 0.0f;
					this->moveCursorY(1);  // NOTE: Doesn't like shifting when inside its own texture mode
				}else{
					this->cursor_position.x = this->max_cursor_position.x;
				}
			}
		}

		// NOTE: Moving cursor vertically may or may not need to scroll as well
		// NOTE: This should work with fractional lines, but is untested
		void moveCursorY(float delta, bool in_lines = true, bool scroll_at_end = true){
			if(in_lines) delta *= this->character_spacing.y;
			this->cursor_position.y += delta;

			if(this->cursor_position.y < 0.0f){
				if(scroll_at_end) this->scrollView(this->cursor_position.y, false);
				this->cursor_position.y = 0.0f;
			}else if(this->cursor_position.y > this->max_cursor_position.y){
				if(scroll_at_end) this->scrollView(this->cursor_position.y - this->max_cursor_position.y, false);
				this->cursor_position.y = this->max_cursor_position.y;
			}
		}

		void scrollView(float delta, bool in_lines = true){
			if(in_lines) delta *= this->character_spacing.y;
			this->scroll_position += delta;

			if(this->scroll_position < 0.0f){
				this->scroll_position = 0.0f;
			}else if(this->scroll_position > this->max_scroll_position){
				util::rendering::Shift(this->history_render_texture, 0, this->max_scroll_position - this->scroll_position, this->current_background_color);  // NOTE: Negative scroll to scroll up
				this->scroll_position = this->max_scroll_position;
			}
		}

		void applyCsiCode(std::vector<CsiParameter> parameters, char final_character){
			switch(final_character){
				case 'A':  // Cursor Up
					if(parameters.size() == 0) this->moveCursorY(-1);
					// TODO: Do this without raising warnings?
					else this->moveCursorY(-nonZeroOr(parameters[0].value, 1));  // NOTE: even explicit 0 is treated as 1
					break;
				case 'B':  // Cursor Down
					if(parameters.size() == 0) this->moveCursorY(1);
					else this->moveCursorY(nonZeroOr(parameters[0].value, 1));
					break;
				case 'C':  // Cursor Forward
					if(parameters.size() == 0) this->moveCursorX(1);
					else this->moveCursorX(nonZeroOr(parameters[0].value, 1));
					break;
				case 'D':  // Cursor Back
					if(parameters.size() == 0) this->moveCursorX(-1);
					else this->moveCursorX(-nonZeroOr(parameters[0].value, 1));
					break;
				case 'E':  // Cursor Next Line
					if(parameters.size() == 0) parameters.push_back(CsiParameter{CsiParameterType::NUMBER, 1});
					this->cursor_position.x = 0.0f;
					this->cursor_position.y += nonZeroOr(parameters[0].value, 1) * this->character_spacing.y;
					break;
				case 'F':  // Cursor Previous Line
					if(parameters.size() == 0) parameters.push_back(CsiParameter{CsiParameterType::NUMBER, 1});
					this->cursor_position.x = 0.0f;
					this->cursor_position.y -= nonZeroOr(parameters[0].value, 1) * this->character_spacing.y;
					break;
				case 'G':  // Cursor Horizontal Absolute
					if(parameters.size() == 0) parameters.push_back(CsiParameter{CsiParameterType::NUMBER, 1});
					this->cursor_position.x = (nonZeroOr(parameters[0].value, 1) - 1) * this->character_spacing.x;
					break;
				case 'H':  // Cursor Position
					switch(parameters.size()){
						case 0:                              // \x1B[H
							parameters.push_back(CsiParameter{CsiParameterType::NUMBER, 1});
						case 1:                              // \x1B[nH
							parameters.push_back(CsiParameter{CsiParameterType::INTERMEDIATE, ';'});
						case 2:
							if(parameters[1].value == (uint16_t)';'){  // \x1B[n;H
								parameters.push_back(CsiParameter{CsiParameterType::NUMBER, 1});
							}else{                           // \x1B[;mH
								this->cursor_position.x = 0.0f;
								this->cursor_position.y = (nonZeroOr(parameters[2].value, 1) - 1) * this->character_spacing.y;
								break;
							}
						case 3:                              // \x1B[n;mH
							this->cursor_position.x = (nonZeroOr(parameters[0].value, 1) - 1) * this->character_spacing.x;
							this->cursor_position.y = (nonZeroOr(parameters[2].value, 1) - 1) * this->character_spacing.y;
							break;
						case 5: // TODO: htop will sometimes send codes like '\x1B[27[3;35H'
							this->cursor_position.x = (nonZeroOr(parameters[2].value, 1) - 1) * this->character_spacing.x;
							this->cursor_position.y = (nonZeroOr(parameters[4].value, 1) - 1) * this->character_spacing.y;
							break;
					}
					break;
				// TODO: Known issue, `clear` command doesn't show prompt on completion
				case 'J':  // Erase in Display
					{
						float height = (float)std::get<1>(this->size());

						if(parameters.size() == 0) parameters.push_back(CsiParameter{CsiParameterType::NUMBER, 0});
						switch(parameters[0].value){
							case 0:  // Clear from cursor to end of screen
								{const modes::TextureMode texture_mode(this->history_render_texture);
									DrawRectangleRec(
										Rectangle{
											this->cursor_position.x, this->scroll_position + this->cursor_position.y,
											this->history_rectangle.width - this->cursor_position.x, this->character_spacing.y
										},
										this->current_background_color
									);
									DrawRectangleRec(
										Rectangle{
											0.0f, this->scroll_position + this->cursor_position.y + this->character_spacing.y,
											this->history_rectangle.width, height - this->cursor_position.y + this->character_spacing.y
										},
										this->current_background_color
									);
								}
								break;
							case 1:  // Clear from cursor to beginning of screen
								{const modes::TextureMode texture_mode(this->history_render_texture);
									// TODO: May not be working
									DrawRectangleRec(
										Rectangle{
											0.0f, this->scroll_position + this->cursor_position.y,
											this->cursor_position.x + this->character_spacing.x, this->character_spacing.y
										},
										this->current_background_color
									);

									DrawRectangleRec(
										Rectangle{
											0.0f, this->scroll_position,
											this->history_rectangle.width, this->cursor_position.y
										},
										this->current_background_color
									);
								}
								break;
							case 2:  // Clear entire screen
								this->scrollView(height, false);
								{const modes::TextureMode texture_mode(this->history_render_texture);
									DrawRectangleRec(
										Rectangle{
											0.0f, this->scroll_position,
											this->history_rectangle.width, height
										},
										this->current_background_color
									);
								}
								break;
							case 3:  // Clear entire buffer (but not the current screen)
								util::rendering::Shift(this->history_render_texture, 0, -this->scroll_position, this->current_background_color);
								this->scroll_position = 0.0f;
								break;
						}
					}
					break;
				case 'K':  // Erase in Line
					if(parameters.size() == 0) parameters.push_back(CsiParameter{CsiParameterType::NUMBER, 0});
					switch(parameters[0].value){
						case 0:  // Clear from cursor to end of line
							{const modes::TextureMode texture_mode(this->history_render_texture);
								DrawRectangleRec(
									Rectangle{
										this->cursor_position.x, this->scroll_position + this->cursor_position.y,
										this->history_rectangle.width - this->cursor_position.x, this->character_spacing.y
									},
									this->current_background_color
								);
							}
							break;
						case 1:  // Clear from cursor to beginning of line
							// TODO: May not be working
							{const modes::TextureMode texture_mode(this->history_render_texture);
								DrawRectangleRec(
									Rectangle{
										0.0f, this->scroll_position + this->cursor_position.y,
										this->cursor_position.x + this->character_spacing.x, this->character_spacing.y
									},
									this->current_background_color
								);
							}
							break;
						case 2:  // Clear entire line
							{const modes::TextureMode texture_mode(this->history_render_texture);
								DrawRectangleRec(
									Rectangle{
										0.0f, this->scroll_position + this->cursor_position.y,
										this->history_rectangle.width, this->character_spacing.y
									},
									this->current_background_color
								);
							}
							break;
					}
					break;
				case 'P':  // Delete Characters
					{
						// TODO: Known issues when deleting with wraparound text, make better
						// Could steganography or similar be used to record when lines wrap vs. newline?
						int characters_to_delete = 1;
						if(parameters.size() > 0) characters_to_delete = nonZeroOr(parameters[0].value, 1);

						float width = this->history_rectangle.width - (characters_to_delete * this->character_spacing.x);
						Texture2D texture = this->current_texture();
						{const modes::TextureMode texture_mode(this->history_render_texture);
							DrawTexturePro(
								texture,
								Rectangle{
									this->cursor_position.x + (characters_to_delete * this->character_spacing.x), this->cursor_position.y,
									width, this->character_spacing.y
								},
								Rectangle{
									this->cursor_position.x, this->scroll_position + this->cursor_position.y,
									width, this->character_spacing.y
								},
								Vector2{0.0f, 0.0f}, 0.0f, WHITE
							);

							DrawRectangleV(Vector2{this->history_rectangle.width - (characters_to_delete * this->character_spacing.x),this->scroll_position+this->cursor_position.y},this->character_spacing,this->current_background_color);
							// TODO: Replace
							DrawRectangleV(Vector2{this->cursor_position.x,this->scroll_position+this->cursor_position.y+12},{this->character_spacing.x,2},this->current_background_color);
						}
					}
					break;
				case 'S':  // Scroll Up
					if(parameters.size() == 0) this->scrollView(-1);
					else this->scrollView(-nonZeroOr(parameters[0].value, 1));
					break;
				case 'T':  // Scroll Down
					if(parameters.size() == 0) this->scrollView(1);
					else this->scrollView(nonZeroOr(parameters[0].value, 1));
					break;
				case 'X':  // Erase Characters
					{
						// TODO: Known issues when deleting with wraparound text, make better
						// Could steganography or similar be used to record when lines wrap vs. newline?
						int characters_to_delete = 1;
						if(parameters.size() > 0) characters_to_delete = nonZeroOr(parameters[0].value, 1);

						float width = this->history_rectangle.width - (characters_to_delete * this->character_spacing.x);
						Texture2D texture = this->current_texture();
						{const modes::TextureMode texture_mode(this->history_render_texture);
							DrawRectangleRec(
								Rectangle{
									this->cursor_position.x, this->cursor_position.y,
									characters_to_delete * this->character_spacing.x, this->character_spacing.y
								},
								this->current_background_color
							);
						}
					}
					break;
				case 'd':  // Line Absolute Position
					this->cursor_position.y = 0.0f;
					if(parameters.size() > 0) this->moveCursorY(nonZeroOr(parameters[0].value, 1) - 1, true, false);
					break;
				// TODO: Get working if allowing pasting
				case 'h':  // Enable bracketed paste - Ignore
				case 'l':  // Disable bracketed paste - Ignore
					break;
				// TODO: Get working?
				case 'r':  // Set Scrolling Region - Ignore
					break;
				case 't':  // Window Manipulation - Ignore
					break;
				case 'm':  // Select Graphic Rendition parameters
					// TODO: Rework to make use of CsiParameter.type property
					for(int i = 0; i < parameters.size(); i += 2){  // NOTE: += 2 to skip semicolons separating parameters
						switch(parameters[i].value){
							case 0:  // Reset
								this->current_foreground_color = this->default_foreground_color;
								this->current_background_color = this->default_background_color;
								this->current_text_effects = NONE;
								break;
							case 1: this->current_text_effects |= INCREASED_INTENSITY; break;
							case 30: this->current_foreground_color = (this->current_text_effects & BRIGHT) ? Color{127, 127, 127, 255} : Color{  0,   0,   0, 255}; break;  // Black Foreground
							case 31: this->current_foreground_color = (this->current_text_effects & BRIGHT) ? Color{255,   0,   0, 255} : Color{205,   0,   0, 255}; break;  // Red Foreground
							case 32: this->current_foreground_color = (this->current_text_effects & BRIGHT) ? Color{  0, 255,   0, 255} : Color{  0, 205,   0, 255}; break;  // Green Foreground
							case 33: this->current_foreground_color = (this->current_text_effects & BRIGHT) ? Color{255, 255,   0, 255} : Color{205, 205,   0, 255}; break;  // Yellow Foreground
							case 34: this->current_foreground_color = (this->current_text_effects & BRIGHT) ? Color{ 92,  92, 255, 255} : Color{  0,   0, 238, 255}; break;  // Blue Foreground
							case 35: this->current_foreground_color = (this->current_text_effects & BRIGHT) ? Color{255,   0, 255, 255} : Color{205,   0, 205, 255}; break;  // Magenta Foreground
							case 36: this->current_foreground_color = (this->current_text_effects & BRIGHT) ? Color{  0, 255, 255, 255} : Color{  0, 205, 205, 255}; break;  // Cyan Foreground
							case 37: this->current_foreground_color = (this->current_text_effects & BRIGHT) ? Color{255, 255, 255, 255} : Color{229, 229, 229, 255}; break;  // White Foreground
							case 38:
								switch(parameters[i += 2].value){
									case 5:  // 8-bit Color Mode
										{
											uint8_t n = parameters[i + 2].value;

											// NOTE: The standard palette options modify the `n` parameter such that the next pass actually handles the color application
											if(n >= 0 && n <= 7){  // Standard foreground color
												parameters[i + 2].value += 30;
											}else if(n >= 8 && n <= 15){  // Bright foreground color
												parameters[i + 2].value += 90;
											}else{
												// NOTE: Because `i` isn't incremented when getting the `n` parameter for the standard palette, it must be incremented now
												i += 2;

												if(n >= 16 && n <= 231){  // 6x6x6 cube colors
													{
														uint8_t red_level = n / 36;
														uint8_t green_level = (n / 6) % 6;
														uint8_t blue_level = n % 6;
														this->current_foreground_color = Color{red_level, green_level, blue_level, 255};
													}
												}else{  // 24-step greyscale (232-255)
													uint8_t grey_level = ((n - 232) * 10) + 8;
													this->current_foreground_color = Color{grey_level, grey_level, grey_level, 255};
												}
											}
										}
										break;
									case 2:  // 24-bit Color Mode
										{
											uint8_t red_level = parameters[i += 2].value;
											uint8_t green_level = parameters[i += 2].value;
											uint8_t blue_level = parameters[i += 2].value;
											this->current_foreground_color = Color{red_level, green_level, blue_level, 255};
										}
										break;
								}
								break;
							case 39: this->current_foreground_color = this->default_foreground_color; break;  // Default Foreground
							case 40: this->current_background_color = Color{  0,   0,   0, 255}; break;  // Black Background
							case 41: this->current_background_color = Color{205,   0,   0, 255}; break;  // Red Background
							case 42: this->current_background_color = Color{  0, 205,   0, 255}; break;  // Green Background
							case 43: this->current_background_color = Color{205, 205,   0, 255}; break;  // Yellow Background
							case 44: this->current_background_color = Color{  0,   0, 238, 255}; break;  // Blue Background
							case 45: this->current_background_color = Color{205,   0, 205, 255}; break;  // Magenta Background
							case 46: this->current_background_color = Color{  0, 205, 205, 255}; break;  // Cyan Background
							case 47: this->current_background_color = Color{229, 229, 229, 255}; break;  // White Background
							case 48:
								switch(parameters[i += 2].value){
									case 5:  // 8-bit Color Mode
										{
											uint8_t n = parameters[i + 2].value;

											// NOTE: The standard palette options modify the `n` parameter such that the next pass actually handles the color application
											if(n >= 0 && n <= 7){  // Standard background color
												parameters[i + 2].value += 40;
											}else if(n >= 8 && n <= 15){  // Bright background color
												parameters[i + 2].value += 100;
											}else{
												// NOTE: Because `i` isn't incremented when getting the `n` parameter for the standard palette, it must be incremented now
												i += 2;

												if(n >= 16 && n <= 231){  // 6x6x6 cube colors
													{
														uint8_t red_level = n / 36;
														uint8_t green_level = (n / 6) % 6;
														uint8_t blue_level = n % 6;
														this->current_background_color = Color{red_level, green_level, blue_level, 255};
													}
												}else{  // 24-step greyscale (232-255)
													uint8_t grey_level = ((n - 232) * 10) + 8;
													this->current_background_color = Color{grey_level, grey_level, grey_level, 255};
												}
											}
										}
										break;
									case 2:  // 24-bit Color Mode
										{
											uint8_t red_level = parameters[i += 2].value;
											uint8_t green_level = parameters[i += 2].value;
											uint8_t blue_level = parameters[i += 2].value;
											this->current_foreground_color = Color{red_level, green_level, blue_level, 255};
										}
										break;
								}
								break;
							case 49: this->current_background_color = this->default_background_color; break;  // Default Background
							case 90: this->current_foreground_color = Color{127, 127, 127, 255}; this->current_text_effects |= BRIGHT; break;  // Bright Black Foreground
							case 91: this->current_foreground_color = Color{255,   0,   0, 255}; this->current_text_effects |= BRIGHT; break;  // Bright Red Foreground
							case 92: this->current_foreground_color = Color{  0, 255,   0, 255}; this->current_text_effects |= BRIGHT; break;  // Bright Green Foreground
							case 93: this->current_foreground_color = Color{255, 255,   0, 255}; this->current_text_effects |= BRIGHT; break;  // Bright Yellow Foreground
							case 94: this->current_foreground_color = Color{ 92,  92, 255, 255}; this->current_text_effects |= BRIGHT; break;  // Bright Blue Foreground
							case 95: this->current_foreground_color = Color{255,   0, 255, 255}; this->current_text_effects |= BRIGHT; break;  // Bright Magenta Foreground
							case 96: this->current_foreground_color = Color{  0, 255, 255, 255}; this->current_text_effects |= BRIGHT; break;  // Bright Cyan Foreground
							case 97: this->current_foreground_color = Color{255, 255, 255, 255}; this->current_text_effects |= BRIGHT; break;  // Bright White Foreground
							case 100: this->current_background_color = Color{127, 127, 127, 255}; break;  // Bright Black Background
							case 101: this->current_background_color = Color{255,   0,   0, 255}; break;  // Bright Red Background
							case 102: this->current_background_color = Color{  0, 255,   0, 255}; break;  // Bright Green Background
							case 103: this->current_background_color = Color{255, 255,   0, 255}; break;  // Bright Yellow Background
							case 104: this->current_background_color = Color{ 92,  92, 255, 255}; break;  // Bright Blue Background
							case 105: this->current_background_color = Color{255,   0, 255, 255}; break;  // Bright Magenta Background
							case 106: this->current_background_color = Color{  0, 255, 255, 255}; break;  // Bright Cyan Background
							case 107: this->current_background_color = Color{255, 255, 255, 255}; break;  // Bright White Background
							default:
								std::cout << "Unhandled SGR parameter: " << parameters[i].value << std::endl;
								break;
						}
					}
					break;
				default:
					// TODO: Temporary
					std::cout << "Unknown escape code: \\x1B[";
					for(int i = 0; i < this->current_csi_parameters.size(); i++){
						if(this->current_csi_parameters[i].type == CsiParameterType::NUMBER){
							std::cout << this->current_csi_parameters[i].value;
						}else{
							std::cout << (char)(this->current_csi_parameters[i].value);
						}
					}
					std::cout << final_character << std::endl;
					break;
			}
		}

		void sendKey(int key){
			if(key >= ' ' && key <= '~'){
				char character = (char)key;
				if(IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)){
					// NOTE: Letters get sent as majiscule, no need to check for miniscule
					// TODO: Support other layouts?
					switch(character){
						case '\'': character = '"'; break;
						case ',': character = '<'; break;
						case '-': character = '_'; break;
						case '.': character = '>'; break;
						case '/': character = '?'; break;
						case '0': character = ')'; break;
						case '1': character = '!'; break;
						case '2': character = '@'; break;
						case '3': character = '#'; break;
						case '4': character = '$'; break;
						case '5': character = '%'; break;
						case '6': character = '^'; break;
						case '7': character = '&'; break;
						case '8': character = '*'; break;
						case '9': character = '('; break;
						case ';': character = ':'; break;
						case '=': character = '+'; break;
						case '[': character = '{'; break;
						case '\\': character = '|'; break;
						case ']': character = '}'; break;
						case '`': character = '~'; break;
					}
				}else{
					if(character >= 'A' && character <= 'Z'){
						character |= 0x20;
					}
				}
				this->ssh_interface.writeChannel(&character, 1);
			}else{
				switch(key){
					case KEY_BACKSPACE: this->ssh_interface.writeChannel("\b", 1); break;
					case KEY_ENTER: this->ssh_interface.writeChannel("\r\n", 2); break;
					case KEY_DELETE: this->ssh_interface.writeChannel("\x7F", 1); break;

					case KEY_LEFT: this->ssh_interface.writeChannel("\x1B[D", 3); break;
					case KEY_RIGHT: this->ssh_interface.writeChannel("\x1B[C", 3); break;
				}
			}
		}
	};
}

#endif
