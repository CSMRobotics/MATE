// https://stackoverflow.com/a/45576888
function onKonamiCode(fnc){
	let input = '';
	let key = '38384040373937396665';
	document.addEventListener('keydown', event => {
		input += "" + event.keyCode;
		if(input === key){
			return fnc();
		}else if(!key.indexOf(input)){
			return;
		}else{
			input = "" + event.keyCode;
		}
	});
}

function makeRotation(x, y, z, theta){
	let normalized_vector = Quaternion([x, y, z]).normalize();
	let c = Math.cos(.5 * theta);
	let s = Math.sin(.5 * theta);
	return new Quaternion(c, s * normalized_vector.x, s * normalized_vector.y, s * normalized_vector.z);
}

$(function(){
	$("#info-tabs-container").tabs();
	$("#shell-input").button()
		.off("mouseenter")
		.off("mousedown")
		.off("keydown")
		.on("keyup", function(key){
			if(key.which == 13){ // Enter
				let command = $(this).val();
				console.log(`Sending command to Flask: ${command}`);

				$.getJSON("/shell",
					$.param({command: command}, true)
				).then((command_result, success, response_object) => {
					console.log(`Recieved command result from Flask (status ${response_object.status}): ${command_result.result}`, response_object);
					$("#shell-output").text(command_result.result);
				}).catch((command_result, success, response_object) => {
					console.error(`Erorr recieving command result from Flask (status ${response_object.status}):`, response_object);
				});
			}
		});

		$(document).keyup(key => {
			if(key.which == 115){ // F4
				$.getJSON("/shell",
					$.param({command: "ros2_node.estop()"}, true)
				);
			}
		});

		let orientation_viewport = new OrientationViewport(document.getElementById("orientation-viewport"));
		onKonamiCode(() => {
			orientation_viewport.use_alt_model = true;
			document.getElementById("graphic").style.opacity = .5;
			console.log("Entering mode: 01");
		});

		let button_active_color = "#2eb398";
		let button_inactive_color = "white";
		let joystick_event_source = new EventSource("/get_joystick");
		joystick_event_source.onmessage = event => {
			let data = JSON.parse(event.data);

			data.axes = data.axes.map(value => 50 * (value + 1));
			data.axes.forEach((value, idx) => {
				$(`#joy-axis-${idx}-bar`).css("width", `${value}%`);
				$(`#joy-axis-${idx}-label`).text(`${~~value}%`);
			});
			data.buttons.forEach((is_active, idx) => {
				$(`#joy-button-${idx + 1}-indicator`).css("background-color", (is_active) ? button_active_color : button_inactive_color);
			});
		};

		let attitude_event_source = new EventSource("/get_attitude");
		attitude_event_source.onmessage = event => {
			let data = JSON.parse(event.data);
			orientation_viewport.orientation_data.current_quaternion = new Quaternion(data.orientation);
			orientation_viewport.orientation_data.current_gravity = data.gravity;
			orientation_viewport.orientation_data.current_magnetometer = data.magnetometer;

			if(orientation_viewport.orientation_data.base_quaternion_inverse === null){
				orientation_viewport.orientation_data.base_quaternion_inverse = orientation_viewport.orientation_data.current_quaternion.inverse();

				let rotated_magnetometer = Quaternion.fromBetweenVectors(
					Object.values(data.gravity),
					[0, 0, -1]
				).rotateVector(Object.values(data.magnetometer));
				let initial_heading = Math.atan2(rotated_magnetometer[1], rotated_magnetometer[0]);
				orientation_viewport.orientation_data.initial_heading = initial_heading;
			}else{
				orientation_viewport.orientation_data.current_delta = orientation_viewport.orientation_data.current_quaternion.mul(orientation_viewport.orientation_data.base_quaternion_inverse).inverse();

				let current_down_array = orientation_viewport.orientation_data.current_delta.rotateVector([0, 0, -1]);
				orientation_viewport.orientation_data.current_down = {
					x: current_down_array[0],
					y: current_down_array[1],
					z: current_down_array[2]
				}
			}
		};
});
