
#include <porszilo_telepresence/telepresence.h>
#include <roscpp>

Telepresence::Telepresence() {
	this._nh.reset(new ros::NodeHandle());
	_nh->param("frequency",     this->FPS, 10);
	_nh->param("view_angle_h",  this->FOV_H, 80);
	_nh->param("view_angle_v",  this->FOV_V, 60);
	_nh->param("mat_rot_vel",   this->MAX_ROT_VEL, 0.5);
	_nh->param("dynamic_map",   this->DYNAMIC_MAP, false);
	_nh->param("canvas_width",  this->CANVAS_W, 700);
	_nh->param("canvas_height", this->CANVAS_H, 500);
}

void Telepresence::run() {
	this-	this->>
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "telepresence");

	Telepresence t;
	t.run();
}

