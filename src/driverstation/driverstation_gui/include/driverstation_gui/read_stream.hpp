#ifndef READ_STREAM_HEADER_INCLUDED
#define READ_STREAM_HEADER_INCLUDED

#include <optional>

namespace read_stream{
	template<typename T>
	struct ReadStream{
		virtual std::optional<T> next() = 0;
		friend ReadStream<T>& operator>>(ReadStream<T>& lhs, std::optional<T>& rhs){
			rhs = lhs.next();
			return lhs;
		};
	};

	template<size_t buffer_capacity>
	class BufferedCharReadStream : public ReadStream<char>{
		public:
			std::optional<char> next() override{
				if(this->current_buffer_index >= this->current_buffer_size){
					this->current_buffer_size = this->refreshBuffer(this->buffer, buffer_capacity);
					this->current_buffer_index = 0;
				}
				if(this->current_buffer_index >= this->current_buffer_size) return std::nullopt;
				return std::optional<char>(this->buffer[this->current_buffer_index++]);
			}

		protected:
			// NOTE: Should return the new buffer size
			virtual size_t refreshBuffer(char* buffer, size_t capacity) = 0;

		private:
			char buffer[buffer_capacity];
			size_t current_buffer_index = 0;
			size_t current_buffer_size = 0;
	};
}

#endif
