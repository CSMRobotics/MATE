class OrientationViewport{
	constructor(container){
		let parent = this;

		this.side_length = 100;

		this.orientation_data = {
			base_quaternion_inverse: null,
			initial_heading: 0,
			current_quaternion: null,
			current_delta: new Quaternion(),
			current_gravity: {
				x: 0,
				y: 0,
				z: 1
			},
			current_magnetometer: {
				x: 1,
				y: 0,
				z: 0
			},
			current_down: {
				x: 0,
				y: 0,
				z: -1
			}
		};

		this.model = null;
		this.alt_model = null;
		this.use_alt_model = false;

		this.sketch = new p5(function(p5){
			let frameCount = 0;

			p5.withState = function(state_name, block){
				p5.push();
				block();
				p5.pop();
			};

			p5.applyQuaternion = function(quaternion){
				p5.applyMatrix(...quaternion.toMatrix4());
			};

			p5.applyRotation = function(x, y, z, theta){
				p5.applyMatrix(...makeRotation(x, y, z, theta).toMatrix4());
			};

			p5.windowResized = function(){
				let jQuery_containing_element = $(p5.canvas.parentElement);
				p5.resizeCanvas(jQuery_containing_element.innerWidth(), jQuery_containing_element.innerHeight());
			}

			p5.preload = function(){
				p5.loadModel(`${static_folder}rov.stl`, true, model => {
					parent.model = model;
				});
				p5.loadModel(`${static_folder}miku.stl`, true, model => {
					parent.alt_model = model;
				});
			}

			p5.setup = function(){
				p5.createCanvas(0, 0, p5.WEBGL);
				p5.windowResized();
			}

			p5.draw = function(){
				p5.background(55, 59, 62);

				p5.withState("orientation_viewport", () => {
					p5.translate(100, 0, 0);
					p5.rotateX(-.4);
					p5.rotateY(.5);
					p5.rotateX(p5.PI / 2);
					p5.rotateZ(1);
					p5.scale(1, -1, 1);
					p5.noFill();

					p5.withState("base_square", () => {
						p5.stroke(255);
						p5.strokeWeight(2);
						p5.translate(0, 0, -parent.side_length);
						p5.box(parent.side_length, parent.side_length, 0);
						p5.withState("base_square_tick_marks", () => {
							let half_side_length = .5 * parent.side_length;
							let half_tick_length = 20;
							let quarter_tick_length = .5 * half_tick_length;
							let tick_start = half_side_length - half_tick_length;
							let tick_end = half_side_length + half_tick_length;

							p5.translate(0, 0, -2);
							p5.stroke(255, 0, 0);
							p5.line(tick_start, 0, tick_end, 0);
							p5.stroke(0, 255, 0);
							p5.line(0, tick_start, 0, tick_end);
							p5.stroke(0, 0, 255);
							p5.line(0, 0, 0, 0, 0, half_tick_length);

							p5.stroke(255);
							p5.strokeWeight(1);
							p5.line(-quarter_tick_length, 0, quarter_tick_length, 0);
							p5.line(0, -quarter_tick_length, 0, quarter_tick_length);
						});
					});

					p5.rotateZ(-parent.orientation_data.initial_heading);
					p5.applyQuaternion(parent.orientation_data.current_delta);
					p5.rotateZ(parent.orientation_data.initial_heading);

					p5.withState("draw_model", () => {
						p5.normalMaterial();
						if(parent.use_alt_model){
							p5.translate(0, -40, 0);
							p5.applyRotation(0, 0, 1, -2.5);
							if(parent.alt_model !== null) p5.model(parent.alt_model);
						}else{
							if(parent.model !== null) p5.model(parent.model);
						}
					});
				});

				p5.withState("flat_ui", () => {
					p5.ortho();
					p5.translate(-200, 0, 100);
					p5.rectMode(p5.CENTER);
					p5.noStroke();
					p5.fill(190, 200, 209);
					p5.rect(0, 0, 200, p5.height);
					p5.withState("artificial_horizon", () => {
						p5.translate(0, -50, 0);
						p5.strokeWeight(3);
						p5.withState("roll_plane", () => {
							p5.noStroke();
							p5.rotate(-p5.atan2(parent.orientation_data.current_down.x, parent.orientation_data.current_down.z));
							p5.fill(134, 206, 203);
							p5.arc(0, 0, 100, 100, 0, p5.PI, p5.OPEN);
							p5.fill(225, 40, 133);
							p5.arc(0, 0, 100, 100, p5.PI, p5.TWO_PI, p5.OPEN);
							p5.withState("pitch_plane", () => {
								let pitch = p5.atan2(parent.orientation_data.current_down.z, parent.orientation_data.current_down.y);
								let mid_color;
								p5.rotateX(pitch);
								if(Math.abs(pitch) <= p5.HALF_PI){
									mid_color = p5.color(134, 206, 203);
								}else{
									mid_color = p5.color(225, 40, 133);
								}
								p5.stroke(mid_color);
								p5.line(-50, 0, 50, 0);
								p5.stroke(19, 122, 127);
								p5.fill(mid_color);
								p5.ellipse(0, 0, 100, 100);

								p5.stroke(55, 59, 62);
								p5.noFill();
								p5.strokeWeight(2);
								for(let angle = -90; angle <= 90; angle += 15){
									if(angle == 0) continue;
									p5.withState("minor_pitch_lines", () => {
										p5.rotateX(p5.radians(angle));
										if(angle % 30 == 0){
											p5.strokeWeight(2);
										}else{
											p5.strokeWeight(1);
										}
										p5.arc(0, 0, 100, 100, -p5.HALF_PI - .25, -p5.HALF_PI + .25);
									});
								}
							});
							p5.stroke(19, 122, 127);
							p5.noFill();
							p5.withState("border", () => {
								p5.translate(0, 0, 50);
								p5.ellipse(0, 0, 100, 100);
							});
						});

						p5.stroke(55, 59, 62);
						p5.strokeWeight(2);
						p5.line(-30, 0, 50, -10, 0, 50);
						p5.line(10, 0, 50, 30, 0, 50);
					});
				});
			}
		}, container);
	}
}
