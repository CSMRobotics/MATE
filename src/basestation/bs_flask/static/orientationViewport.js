class OrientationViewport{
	constructor(container){
		let parent = this;
		this._setupDone = false;

		this.sketch = new p5(function(p5){
			let frameCount = 0;

			p5.windowResized = function(){
				let jQuery_containing_element = $(p5.canvas.parentElement);
				p5.resizeCanvas(jQuery_containing_element.innerWidth(), jQuery_containing_element.innerHeight());
			}

			p5.setup = function(){
				p5.createCanvas(0, 0, p5.WEBGL);
				p5.windowResized();
				parent._onP5Setup();
			}

			p5.draw = function(){
				p5.background(0);
				p5.rotateX(-.4);
				p5.rotateY(.5);
				p5.noFill();

				p5.stroke(255, 0, 0);
				p5.line(0, 0, 0, 70, 0, 0);
				p5.stroke(0, 255, 0);
				p5.line(0, 0, 0, 0, -70, 0);
				p5.stroke(0, 0, 255);
				p5.line(0, 0, 0, 0, 0, 70);

				p5.stroke(255);
				p5.push();
				p5.rotateY(frameCount * -0.01);
				p5.box(150, 150, 150);
				p5.pop();
				frameCount++;
			}
		}, container);
	}

	_onP5Setup(){
		this.canvas = this.sketch.canvas;
		this.container_element = this.canvas.parentElement;

		this._setupDone = true;
	}
}