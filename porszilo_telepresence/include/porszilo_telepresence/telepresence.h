#ifndef TELEPRESENCE
#define TELEPRESENCE

#include <roscpp>
#include <tf/transform_listener.h>
#include <cmath>
#include <memory>

// message types
#include <sensor_msgs/Image.h>
#include <nav_msgs/OccupancyGrid.h>
#include <porszilo_telepresence/ClickedPoint.h>
#include <porszilo_telepresence/CanvasSize.h>
#include <std_msgs/String.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/Point.h>

class Telepresence() {
// - get clicked point position from browser
// - convert 2d point to 3d
// - check if the point if free on the costmap
//   - if free, send the goal to move_base, and tell the browser 
//   	where to draw the marker (3d to 2d)
//   - if occupied, tell the browser
//
// - if user wants to rotate, send message to move_base


private:
	std::shared_ptr<ros::NodeHandle> _nh;
	ros::Subscriber subCostmap;
	ros::Subscriber subGoalStatus;
	ros::Publisher pubGoal;
	tf::TransformListener listener;

	float FPS, FOV_H, FOV_V, MAX_ROT_VEL, CANVAS_W, CANVAS_H;
	bool DYNAMIC_MAP;
	geometry_msgs::Point to3d(geometry_msgs::Point pt);
	geometry_msgs::Point to2d(geometry_msgs::Point pt);


public:
	Telepresence();
		// - get parameters
		// 		fps
		// 		fov_h
		// 		fov_v
		// 		max_rot_vel
		// 		dynamic_map
		// 		canvas_w
		// 		canvas_h

	void run();
		// - set up subscribers
		// 		/move_base/global_costmap/costmap
		// 		/move_base/status
		// 		/tf map camera_link
		// - set up publishers
		// 		/porszilo/move_base/goal
		// 		
};

#endif
