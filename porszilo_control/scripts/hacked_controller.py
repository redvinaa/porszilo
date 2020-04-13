#! /usr/bin/env python

import rospy
import tf
from time              import time
from geometry_msgs.msg import Twist
from gazebo_msgs.msg   import ModelStates, ModelState
from math              import sin, cos


def lst_to_quat(q):
    class quat():
        x = q[0]
        y = q[1]
        z = q[2]
        w = q[3]
    return quat()

class HackedController():

    def cbTwistMux(self, data):

        dt = time() - self.start_time
        self.start_time = time()

        if data.linear.x > self.max_vel_x:
            data.linear.x = self.max_vel_x
        if data.linear.y > self.max_vel_y:
            data.linear.y = self.max_vel_y
        if data.angular.z > self.max_vel_z:
            data.angular.z = self.max_vel_z
        
        quaternion = (
            self.state.pose.orientation.x,
            self.state.pose.orientation.y,
            self.state.pose.orientation.z,
            self.state.pose.orientation.w)
        yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
        yaw += dt * data.angular.z

        x0 = data.linear.x
        y0 = data.linear.y
        vx0 = (x0*cos(yaw)+y0*sin(yaw)) * cos(yaw)
        vy0 = (x0*cos(yaw)+y0*sin(yaw)) * sin(yaw)

        self.state.pose.position.x += dt * vx0
        self.state.pose.position.y += dt * vy0

        self.state.pose.orientation = \
            lst_to_quat(tf.transformations.quaternion_from_euler(0, 0, yaw))

        self.state.twist = data
        self.gazPub.publish(self.state)

    def run(self):
        self.state = ModelState()

        rospy.init_node('hacked_controller', disable_signals=True)

        self.max_vel_x = rospy.get_param("/mobile/base/controller/linear/x/max_velocity", 1)
        self.max_vel_y = rospy.get_param("/mobile/base/controller/linear/y/max_velocity", 1)
        self.max_vel_z = rospy.get_param("/mobile/base/controller/angular/z/max_velocity", 1)

        rospy.loginfo("waiting for initial pose...")
        while True:
            msg = rospy.wait_for_message("/gazebo/model_states", ModelStates)
            for i in range(len(msg.name)):
                if msg.name[i] == "porszilo":
                    self.state.model_name = "porszilo"
                    self.state.reference_frame = "world"
                    self.state.pose  = msg.pose[i]
                    self.state.twist = msg.twist[i]
                    self.start_time = time()
                    break
            if self.state.model_name == "porszilo":
                break
            else:
                rospy.logwarn("No model named 'porszilo'")

        rospy.loginfo("received initial pose")
        self.start_time = time()

        rospy.Subscriber("/twist_muxed/cmd_vel", Twist, self.cbTwistMux)
        self.gazPub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=5)

        rospy.spin()
        rospy.signal_shutdown("hacked_controller is being shut down")


if __name__ == "__main__":
    ctrl = HackedController()
    ctrl.run()
