function resizeImages () {
	document.getElementById("screen").style.width = "50%";
	document.getElementById("grid").style.width = "50%";
}

// window.addEventListener('resize', resizeImages );

window.onload = function () {

// resizeImages();

ros = new ROSLIB.Ros({
	url: "ws://localhost:9090"
});

ros.on('connection', function() {
	console.log('Connected to websocket server');
});

ros.on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);
	alert('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
  console.log('Connection to websocket server closed.');
});


var srvClick = new ROSLIB.Service({
	ros: ros,
	name: "/telepresence/click",
	serviceType: "porszilo_telepresence_v2/Click"
});

var srvRotate = new ROSLIB.Service({
	ros: ros,
	name: "/telepresence/rotate",
	serviceType: "porszilo_telepresence_v2/Rotate"
});



var pubCancelGoal = new ROSLIB.Topic({
	ros: ros,
	name: '/move_base/cancel',
	messageType: 'actionlib_msgs/GoalID'
});
var cancelMessage = new ROSLIB.Message({});

$("#cancel").on('click', function(event) {
	pubCancelGoal.publish(cancelMessage);
});

var subGrid = new ROSLIB.Topic({
	ros: ros,
	name: '/rtabmap/grid_prob_map',
	messageType: 'nav_maga/OccupancyGrid'
});
subGrid.subscribe(function(data) {
	var grid = document.getElementById("grid");
	grid.style.height = String(grid.style.width/data.info.width*data.info.height);
});


// manage buttons
var pubTurnButtons = new ROSLIB.Service({
	ros: ros,
	name: "/telepresence/rotate",
	messageType: "porszilo_telepresence_v2/Rotate"
})

var btn_left       = document.getElementById("left");
var btn_right      = document.getElementById("right");
var btn_color_orig = btn_left.style.background;
var btn_color_down = "white"
var btn_msg        = new ROSLIB.ServiceRequest()

btn_left.onmousedown = () => {
	btn_msg.left = true;
	btn_msg.on = true;
	pubTurnButtons.callService(btn_msg)
	btn_left.style.background = btn_color_down;
}
btn_left.onmouseup = () => {
	btn_msg.on = false;
	pubTurnButtons.callService(btn_msg)
	btn_left.style.background = btn_color_orig;
}

btn_right.onmousedown = () => {
	btn_msg.left = false;
	btn_msg.on = true;
	pubTurnButtons.callService(btn_msg)
	btn_right.style.background = btn_color_down;
}
btn_right.onmouseup = () => {
	btn_msg.on = false;
	pubTurnButtons.callService(btn_msg)
	btn_right.style.background = btn_color_orig;
}



el = document.getElementById("screen");
if (!el) {throw "Could not get canvas element"};


el.onclick = function(event) {
	var x = (event.pageX - el.offsetLeft) / el.width;
	var y = (event.pageY - el.offsetTop) / el.height;
	
	var request = new ROSLIB.ServiceRequest({
		x: x,
		y: y
	});
	
	console.log("Sending request...");

	var inv_pos = document.getElementById("invalid_pos");

	srvClick.callService(request, function(result) {
		console.log("Calling service " + srvClick.name + ", success: " + result.success);
		if (!result.success) {
			// inv_pos.style.display = "block";
			// setTimeout(()=>{inv_pos.style.display="none"}, 2000);
			alert("Invalid click")
		}
	});
}

el.onmousedown = function(event) {
	setTimeout(function() {
		
}, 500);
}

el.onhover = function(event) {
	setTimeout()
}

}
