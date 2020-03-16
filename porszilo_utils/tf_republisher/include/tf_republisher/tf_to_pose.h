
#ifndef TF_TO_POSE
#define TF_TO_POSE

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include <string>
#include <memory> // std::shared_ptr

namespace tf_republisher {

/* This is a node to republish tf messages as nav_msgs/Poseetry
 * written by Vince Reda - redvinaa@gmail.com
 */

class TfToPose {
public:
	TfToPose();
	void run();
private:
	std::shared_ptr<ros::NodeHandle> nh;
	ros::Publisher                   posePub;
	tf::TransformListener            listener;
	geometry_msgs::TransformStamped  transform;
	geometry_msgs::PoseStamped       pose;

	// parameters
	std::string parent_frame;
	std::string child_frame;
	std::string tf_prefix;
	std::string topic_out;
	double      frequency;

	void getParams();
	void fillPose();
};

}

#endif // TF_TO_POSE
