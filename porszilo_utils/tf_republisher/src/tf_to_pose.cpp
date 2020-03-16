
#include <tf_republisher/tf_to_pose.h>

using namespace tf_republisher;

TfToPose::TfToPose()
	: parent_frame("map")
	, child_frame("odom")
	, tf_prefix("")
	, topic_out("pose")
	, frequency(10.0) {

	nh.reset(new ros::NodeHandle());
	getParams();

	if (tf_prefix != "") {
		parent_frame = tf_prefix + "/" + parent_frame;
		child_frame  = tf_prefix + "/" + child_frame;
	}

	pose.header.frame_id = child_frame;
}


void TfToPose::getParams() {
	nh->param("parent_frame", parent_frame, parent_frame);
	nh->param("child_frame",  child_frame,  child_frame);
	nh->param("tf_prefix",    tf_prefix,    tf_prefix);
	nh->param("topic_out",    topic_out,    topic_out);
	nh->param("frequency",    frequency,    frequency);
}


void TfToPose::run() {

	posePub = nh->advertise<geometry_msgs::PoseStamped>(topic_out, 10);

	ros::Rate rate(frequency);
	while (ros::ok()) {
		try {
			tf::StampedTransform stampedTF;
			listener.lookupTransform(child_frame, parent_frame, ros::Time(0), stampedTF);
			tf::transformStampedTFToMsg(stampedTF, transform);
		}
		catch (tf::TransformException ex) {
			ROS_ERROR_STREAM(ex.what());
			usleep(1e+6);
			continue;
		}
		fillPose();

		posePub.publish(pose);
		rate.sleep();
	}
}


void TfToPose::fillPose() {
	// pose
	pose.header.stamp = ros::Time(0);
	pose.pose.position.x = transform.transform.translation.x;
	pose.pose.position.y = transform.transform.translation.y;
	pose.pose.position.z = transform.transform.translation.z;
	pose.pose.orientation.x = transform.transform.rotation.x;
	pose.pose.orientation.y = transform.transform.rotation.y;
	pose.pose.orientation.z = transform.transform.rotation.z;
	pose.pose.orientation.w = transform.transform.rotation.w;

}




int main(int argc, char** argv) {

ros::init(argc, argv, "tf_to_pose");

TfToPose mom;
mom.run();

}
