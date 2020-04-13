window.onload = function () {

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

$("#cancel_goal").on('click', function(event) {
	pubCancelGoal.publish(cancelMessage);
});




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
			inv_pos.style.display = "block";
			setTimeout(()=>{inv_pos.style.display="none"}, 2000);
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
