#! /usr/bin/env python

import rospy
import tf
from math import *
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import rospkg
import os
import sys
import traceback

# message types
from sensor_msgs.msg              import Image, CameraInfo
from nav_msgs.msg                 import OccupancyGrid
from porszilo_telepresence_v2.srv import Click, ClickResponse, Rotate, RotateResponse
from actionlib_msgs.msg           import GoalStatusArray, GoalID
from geometry_msgs.msg            import Point, PoseStamped, Twist

def Rot(phi):
    return np.array([[cos(phi), -sin(phi)], [sin(phi), cos(phi)]])


class ImgTransform():
    def __init__(self, **kwargs):
        self.Sv = kwargs["Sv"]
        self.Sh = kwargs["Sh"]
        self.PhiV = kwargs["PhiV"]
        self.PhiH = kwargs["PhiH"]
        self.h = kwargs["h"]
        self.MAX = kwargs["MAX"]

        self.dv = self.Sv/2/tan(self.PhiV/2)
        self.dh = self.Sh/2/tan(self.PhiH/2)

        self.D = self.h/tan(self.PhiV/2)

        self.tf_listener = tf.TransformListener()

    def to2D(self, pt, global_coords=True):
        # if cam_pose is provided, pt is given in global coords.
        # (cam_pose = global_coords -> cam)

        if global_coords:
            pose = PoseStamped()
            pose.pose.position.x = pt.x
            pose.pose.position.y = pt.y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            pose.header.frame_id = "/map"
    
            while True:
                try:
                    pose = self.tf_listener.transformPose("/camera_link", pose)
                    break
                except:
                    rospy.logerr("ERROR to2D")
                    rospy.sleep(0.5)
            pt.x = pose.pose.position.x
            pt.y = pose.pose.position.y

        ST = pt.x * tan(self.PhiH/2)
        assert(self.D < pt.x)
        assert(pt.x < self.MAX)
        assert(-ST < pt.y)
        assert(pt.y < ST)

        ret = Point()
        ret.x = (self.Sh/2 - pt.y * self.dh / pt.x) *2
        ret.y = self.h * self.dv / pt.x
        ret.y = self.Sv + self.h * self.dv / pt.x


        return ret

    def to3D(self, pt, global_coords=True):
        # if cam_pose is provided, output is given in global coords.
        # (cam_pose = global_coords -> cam)

        assert(0 < pt.x)
        assert(pt.x < self.Sh)
        assert(self.Sv/2 < pt.y)
        assert(pt.y < self.Sv)

        ret = Point()
        ret.x = self.h * self.dv / (pt.y - self.Sv/2)
        ret.y = ret.x/self.dh * (self.Sh/2 - pt.x)

        if global_coords:
            pose = PoseStamped()
            pose.pose.position.x = ret.x
            pose.pose.position.y = ret.y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            pose.header.frame_id = "/camera_link"
    
            while True:
                try:
                    pose = self.tf_listener.transformPose("/map", pose)
                    break
                except:
                    rospy.logerr("ERROR to3D")
                    rospy.sleep(0.5)
            pt.x = pose.pose.position.x
            pt.y = pose.pose.position.y
            ret = pt

        return ret


class Telepresence():



    def __init__(self):
        rospy.init_node("telepresence")

        self.FPS = rospy.get_param("fps", 10)
        self.FOV_V = radians(rospy.get_param("fov_v", 60))
        self.FOV_H = radians(rospy.get_param("fov_h", 80))
        self.ROTATE_ANGLE = rospy.get_param("rotate_angle", .7)
        self.ROTATE_TIME = rospy.get_param("rotate_time", 5)
        self.GRID_NUM = rospy.get_param("grid_num", 8)
        self.GRID_DIST = rospy.get_param("grid_dist", 10)
        self.DYNAMIC_MAP = rospy.get_param("dynamic_map", True)

        self.camera_info = rospy.wait_for_message("/camera/camera_info", CameraInfo)
        self.rot_mat = np.reshape(self.camera_info.P, (3, 4))

        self.cvBridge = CvBridge()
        self.tf_listener = tf.TransformListener()
        self.goal_on = False
        self.goal = PoseStamped()
        self.goal.header.frame_id = "map"
        self.goal.header.stamp = rospy.Time.now()
        self.rotate = Rotate()
        self.rotate.on = False
        self.br = tf.TransformBroadcaster()

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('porszilo_telepresence_v2')
        self.arrow_pic = cv2.imread(os.path.join(pkg_path, "arrow.png"))
        self.arrow_pic = 255 - cv2.resize(self.arrow_pic, (0,0), fx=0.04, fy=0.04)
        self.marker_pic = cv2.imread(os.path.join(pkg_path, "marker.png"))
        self.marker_pic = 255 - cv2.resize(self.marker_pic, (0,0), fx=0.08, fy=0.08)

        rospy.logwarn('Waiting for messages to arrive...')
        try:
            self.cbColor(rospy.wait_for_message("/camera/color/image_raw", Image))
            self.cbDepth(rospy.wait_for_message("/camera/depth/image_raw", Image))
            self.cbGrid(rospy.wait_for_message("/rtabmap/grid_prob_map", OccupancyGrid))
            while self.grid.data[1:] == self.grid.data[:-1]:
                rospy.sleep(1)
                self.cbGrid(rospy.wait_for_message("/rtabmap/grid_prob_map", OccupancyGrid))
            self.getTransform()


        except rospy.ROSInterruptException:
            rospy.loginfo("Telepresence node is being shut down")
            quit()

        self.imgTransform = ImgTransform(
            Sv = self.camera_info.height,
            Sh = self.camera_info.width,
            PhiV = self.FOV_V,
            PhiH = self.FOV_H,
            h = self.tf_pose[0][2],
            MAX = 10
        )

    def run(self):

        rospy.logwarn('The messages have arrived')

        rospy.Subscriber("/move_base/status", GoalStatusArray, self.cbStatus)
        rospy.Subscriber("/camera/color/image_raw", Image, self.cbColor)
        rospy.Subscriber("/camera/depth/image_raw", Image, self.cbDepth)

        if self.DYNAMIC_MAP:
            rospy.Subscriber("/rtabmap/grid_prob_map", OccupancyGrid, self.cbGrid)

        rospy.Service("/telepresence/click", Click, self.cbClick)
        rospy.Service("/telepresence/rotate", Rotate, self.cbRotate)

        self.pubImage = rospy.Publisher("/telepresence/image", Image, queue_size=1)
        self.pubGrid = rospy.Publisher("telepresence/grid", Image, queue_size=1)
        self.pubGoal = rospy.Publisher("/porszilo/move_base/goal", PoseStamped, queue_size=5)
        self.pubCancelGoal = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)


        rate = rospy.Rate(self.FPS)
        try:
            cnt = 1
            while not rospy.is_shutdown():
                self.getTransform()

                self.image = self.cv_color

                self.doRotate()
                #  self.placePoints()
                self.placeMarker()
                #  self.pubGridImage()
                self.pubImage.publish(self.cvBridge.cv2_to_imgmsg(self.image, encoding="bgr8"))

                rate.sleep()
        except rospy.ROSInterruptException:
            rospy.signal_shutdown("Telepresence node is being shut down")



    def cbStatus(self, data):
        self.goal_on = False
        try:
            if data.status_list[-1].status != 3:
                self.goal_on = True
        except:
            pass # False

    def cbColor(self, data):
        self.cv_color = self.cvBridge.imgmsg_to_cv2(data)

    def cbDepth(self, data):
        self.cv_depth = self.cvBridge.imgmsg_to_cv2(data)

    def cbGrid(self, data):
        self.grid = data

    def getTransform(self):
        while True:
            try:
                #  self.tf_pose = self.tf_listener.lookupTransform('/camera_link', '/map', rospy.Time(0))
                self.tf_pose = self.tf_listener.lookupTransform('/map', '/camera_link', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
                rospy.logerr("ERROR getTransform")
                rospy.sleep(0.5)
        while True:
            try:
                #  self.tf_base_to_cam = self.tf_listener.lookupTransform('/camera_link', '/base_footprint', rospy.Time(0))
                self.tf_base_to_cam = self.tf_listener.lookupTransform('/base_footprint', '/camera_link', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
                rospy.logerr("ERROR getTransform")
                rospy.sleep(0.5)



    def cbClick(self, data):
        pt = Point()
        pt.x = int(data.x * self.camera_info.width)
        pt.y = int(data.y * self.camera_info.height)

        res = ClickResponse()
        
        try:
            pt3D = self.imgTransform.to3D(pt, True)
        except:
            self.cancelGoal()
            res.success = False
            return res

        if not self.checkGrid(pt3D):
            self.cancelGoal()
            return res

        pose = PoseStamped()
        pose.pose.position.x = pt3D.x
        pose.pose.position.y = pt3D.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        pose.header.frame_id = "/map"
    
        pose = self.tf_listener.transformPose("/camera_link", pose)
        dx = pose.pose.position.x
        dy = pose.pose.position.y

        self.goal.pose.position = pt3D

        quaternion = (
            self.tf_pose[1][0],
            self.tf_pose[1][1],
            self.tf_pose[1][2],
            self.tf_pose[1][3])
        euler = tf.transformations.euler_from_quaternion(quaternion)

        yaw = atan(dy/dx) + euler[2]
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        self.goal.pose.orientation.x = quaternion[0]
        self.goal.pose.orientation.y = quaternion[1]
        self.goal.pose.orientation.z = quaternion[2]
        self.goal.pose.orientation.w = quaternion[3]

        self.pubGoal.publish(self.goal)

        res.success = True
        return res

    def cbRotate(self, data):
        self.rotate = data


    def doRotate(self):
        if not self.rotate.on:
            return

        quaternion = (
            self.tf_pose[1][0],
            self.tf_pose[1][1],
            self.tf_pose[1][2],
            self.tf_pose[1][3])
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]

        epsilon = 0.0
        self.goal.pose.position.x = self.tf_pose[0][0] + epsilon*cos(yaw)
        self.goal.pose.position.y = self.tf_pose[0][1] + epsilon*sin(yaw)
        self.goal.pose.position.z = self.tf_pose[0][2]

        if self.rotate.left:
            yaw += self.ROTATE_ANGLE
        else:
            yaw -= self.ROTATE_ANGLE

        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        self.goal.pose.orientation.x = quaternion[0]
        self.goal.pose.orientation.y = quaternion[1]
        self.goal.pose.orientation.z = quaternion[2]
        self.goal.pose.orientation.w = quaternion[3]

        self.pubGoal.publish(self.goal)

    def placePoints(self):
        for i in range(self.GRID_NUM):
            for j in range(self.GRID_NUM):
                p = Point()
                p.x = self.tf_pose[0][0] + (i-self.GRID_NUM/2) * self.GRID_DIST/4.
                p.y = self.tf_pose[0][1] + (j-self.GRID_NUM/2) * self.GRID_DIST/4.
                if self.checkGrid(p):
                    try:
                        p = self.imgTransform.to2D(p, True)
                    except:
                        continue
                    self.image = self.overlayImage(self.image, self.marker_pic, p)

        


    def placeMarker(self):
        if not self.goal_on:
            return

        pt = Point()
        pt.x = self.goal.pose.position.x
        pt.y = self.goal.pose.position.y
        try:
            pt = self.imgTransform.to2D(pt, True)
            #  pos = (pt.x-self.marker_pic.shape[0]/2, pt.y-self.marker_pic.shape[1]*1.5)
            pos = (pt.x-self.marker_pic.shape[0]/2, pt.y-self.marker_pic.shape[1]*2)

            self.image.setflags(write=1)
            self.image = self.overlayImage(self.image, self.marker_pic, pos)
        except:
            pass


    def pubGridImage(self):

        grid = self.grid
        grid_img = np.reshape(grid.data, (grid.info.height, grid.info.width))
        for i in range(grid_img.shape[0]):
            for j in range(grid_img.shape[1]):
                if grid_img[i, j] == -1:
                    grid_img[i, j] = 0.0
        grid_img = 1.0 - np.array(grid_img).astype("uint8") / 100.0
        grid_img = cv2.flip(grid_img, 0)

        Vm_p = np.array([self.tf_pose[0][0], self.tf_pose[0][1]])
        Vm_o = np.array([grid.info.origin.position.x, grid.info.origin.position.y])
        Vo_p = Vm_p - Vm_o

        Vo_p /= grid.info.resolution
        pos = (int(Vo_p[0]), int(grid_img.shape[0]-Vo_p[1]))

        quaternion = (
            self.tf_pose[1][0],
            self.tf_pose[1][1],
            self.tf_pose[1][2],
            self.tf_pose[1][3])
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = -euler[2]
        length = 12
        pos2 = (int(pos[0]+length*cos(yaw)), int(pos[1]+length*sin(yaw)))

        grid_img = cv2.arrowedLine(grid_img, pos, pos2, 0, 1, 8, 0, 0.3)
        grid_img = self.cvBridge.cv2_to_imgmsg(grid_img, encoding="passthrough")

        self.pubGrid.publish(grid_img)


    def checkGrid(self, pt):# needs 3d points in global coords. (geometry_msgs/Point)

        grid = self.grid
        pos_x = pt.x - grid.info.origin.position.x
        pos_y = pt.y - grid.info.origin.position.y
        pos_x /= grid.info.resolution
        pos_y /= grid.info.resolution
        pos_x = int(pos_x)
        pos_y = int(pos_y)

        idx = grid.info.width * pos_y + pos_x
        try:
            data = self.grid.data[idx]
        except:
            return False
        return data < 50

    def cancelGoal(self):
        empty = GoalID()
        self.pubCancelGoal.publish(empty)

    def overlayImage(self, big, small, pos):
        #  pos = list(pos)

        for i in range(small.shape[0]):
            for j in range(small.shape[1]):
                if sum(small[i, j, :]) > 100:
                    try:
                        big[i+int(pos[1]), j+int(pos[0]), :] = small[i, j, :]
                    except:
                        pass
        return big


if __name__ == "__main__":
    t = Telepresence()
    t.run()
