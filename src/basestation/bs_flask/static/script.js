// const base64_data = {'A': 0, 'B': 1, 'C': 2, 'D': 3, 'E': 4, 'F': 5, 'G': 6, 'H': 7, 'I': 8, 'J': 9, 'K': 10, 'L': 11, 'M': 12, 'N': 13, 'O': 14, 'P': 15, 'Q': 16, 'R': 17, 'S': 18, 'T': 19, 'U': 20, 'V': 21, 'W': 22, 'X': 23, 'Y': 24, 'Z': 25, 'a': 26, 'b': 27, 'c': 28, 'd': 29, 'e': 30, 'f': 31, 'g': 32, 'h': 33, 'i': 34, 'j': 35, 'k': 36, 'l': 37, 'm': 38, 'n': 39, 'o': 40, 'p': 41, 'q': 42, 'r': 43, 's': 44, 't': 45, 'u': 46, 'v': 47, 'w': 48, 'x': 49, 'y': 50, 'z': 51, '0': 52, '1': 53, '2': 54, '3': 55, '4': 56, '5': 57, '6': 58, '7': 59, '8': 60, '9': 61, '+': 62, '/': 63};

// Emperical Estimate of .12151 microseconds per pixel conversion
// Dividing 8000000 by the number of pixels in a frame should give a rough estimate of maximum framerate possible with this conversion rate

// https://stackoverflow.com/questions/14446447/how-to-read-a-local-text-file#14446538
// function readTextFile(file, callback){
// 	var rawFile = new XMLHttpRequest();
// 	rawFile.open("GET", file, false);
// 	rawFile.onreadystatechange = function(){
// 		if(rawFile.readyState === 4){
// 			if(rawFile.status === 200 || rawFile.status == 0){
// 				var allText = rawFile.responseText;
// 				callback(allText);
// 			}
// 		}
// 	}
// 	rawFile.send(null);
// }

// let test_metadata = {};
// let test_data;
// readTextFile("200x200p10_servo.storm-video", data => {
// 	let metadata_line;
// 	[metadata_line, ...test_data] = data.split('\n');
//
// 	for(metadata_tag of metadata_line.split(',')){
// 		[key, value] = metadata_tag.split('=');
// 		test_metadata[key] = Number(value);
// 	}
// });

// while(test_data == undefined){};

// let camera1 = new CameraViewport("camera-1-viewport");
// let camera2 = new CameraViewport("camera-2-viewport");
// let camera3 = new CameraViewport("camera-3-viewport");
// let camera4 = new CameraViewport("camera-4-viewport");

// let orientationView = new OrientationViewport("orientation-viewport");

// let frame_number = 0;
// let durations = [];
// setInterval(function(){
// 	let start = new Date();
// 	frame_number++;
// 	if(frame_number >= test_metadata.frame_count){
// 		frame_number = 0;
// 	}
	// frame_data = test_data[frame_number];
	// camera1.drawImage(frame_data, test_metadata.width, test_metadata.height);
	// camera2.drawImage(frame_data, test_metadata.width, test_metadata.height);
	// camera3.drawImage(frame_data, test_metadata.width, test_metadata.height);
	// camera4.drawImage(frame_data, test_metadata.width, test_metadata.height);
// 	durations.push(new Date() - start);
// }, 1000 / test_metadata.framerate);





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
				})
			}
		});

		$(document).keyup(key => {
			if(key.which == 115){ // F4
				$.getJSON("/shell",
					$.param({command: "exit()"}, true)
				)
			}
		})
});
