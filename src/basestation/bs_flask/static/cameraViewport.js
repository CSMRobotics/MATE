class CameraViewport{
	constructor(container){
		let parent = this;
		this._setupDone = false;

		this.sketch = new p5(function(p5){
			p5.drawImage = function(image_data, image_width, image_height){
				let image = p5.createImage(image_width, image_height);

				let pixel = 0;
				let pixel_data_iterator = image_data.matchAll(".{2}");
				let pixel_data = pixel_data_iterator.next();

				image.loadPixels();
				while(pixel_data?.value != undefined){
					pixel_data = pixel_data.value[0];

					let first_char = base64_data[pixel_data[0]];
					let second_char = base64_data[pixel_data[1]];

					let red = (first_char >> 2) << 4;
					let green = (((first_char % 4) << 2) + (second_char >> 4)) << 4;
					let blue = (second_char % 16) << 4;	

					image.pixels[pixel + 0] = red;
					image.pixels[pixel + 1] = green;
					image.pixels[pixel + 2] = blue;
					image.pixels[pixel + 3] = 255;

					pixel_data = pixel_data_iterator.next();
					pixel += 4;
				}
				image.updatePixels();

				p5.image(image, 0, 0, p5.canvas.width, p5.canvas.height);
			}

			p5.windowResized = function(){
				let jQuery_containing_element = $(p5.canvas.parentElement);
				p5.resizeCanvas(jQuery_containing_element.innerWidth(), jQuery_containing_element.innerHeight());
			}

			p5.setup = function(){
				p5.createCanvas(0, 0);
				p5.windowResized();
				parent._onP5Setup();
			}
		}, container);
	}

	_onP5Setup(){
		this.canvas = this.sketch.canvas;
		this.container_element = this.canvas.parentElement;

		this._setupDone = true;
	}

	drawImage(image_data, image_width, image_height){
		this.sketch.drawImage(image_data, image_width, image_height);
	}
}