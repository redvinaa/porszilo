#! /usr/bin/env python

import rospy
import tf
from math import *
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import rospkg
import os

# message types
from sensor_msgs.msg              import Image, CameraInfo
from nav_msgs.msg                 import OccupancyGrid
from porszilo_telepresence_v2.srv import Click, ClickResponse, Rotate, RotateResponse
from actionlib_msgs.msg           import GoalStatusArray, GoalID
from geometry_msgs.msg            import Point, PoseStamped, Twist


class Telepresence():

    def __init__(self):
        rospy.init_node("telepresence")

        self.FPS = rospy.get_param("fps", 10)
        self.ROTATE_ANGLE = rospy.get_param("rotate_angle", 1)
        self.ROTATE_TIME = rospy.get_param("rotate_time", 5)
        self.GRID_W = rospy.get_param("grid_w", 10)
        self.GRID_H = rospy.get_param("grid_h", 10)
        self.DYNAMIC_MAP = rospy.get_param("dynamic_map", False)

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

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('porszilo_telepresence_v2')
        self.arrow_pic = cv2.imread(os.path.join(pkg_path, "arrow.png"))
        self.arrow_pic = 255 - cv2.resize(self.arrow_pic, (0,0), fx=0.04, fy=0.04)
        self.marker_pic = cv2.imread(os.path.join(pkg_path, "marker.png"))
        self.marker_pic = 255 - cv2.resize(self.marker_pic, (0,0), fx=0.2, fy=0.2)

        rospy.logwarn('Waiting for messages to arrive...')
        try:
            self.cbColor(rospy.wait_for_message("/camera/color/image_raw", Image))
            self.cbDepth(rospy.wait_for_message("/camera/depth/image_raw", Image))
            self.cbGrid(rospy.wait_for_message("/move_base/global_costmap/costmap", OccupancyGrid))
            while self.grid.data[1:] == self.grid.data[:-1]:
                rospy.sleep(1)
                self.cbGrid(rospy.wait_for_message("/move_base/global_costmap/costmap", OccupancyGrid))
            while True:
                try:
                    self.tf_pose = self.tf_listener.lookupTransform('/camera_link', '/map', rospy.Time(0))
                    break
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
                    print(ex)
                    rospy.sleep(0.5)

        except rospy.ROSInterruptException:
            rospy.loginfo("Telepresence node is being shut down")
            quit()

    def run(self):

        rospy.logwarn('The messages have arrived')

        rospy.Subscriber("/move_base/status", GoalStatusArray, self.cbStatus)
        rospy.Subscriber("/camera/color/image_raw", Image, self.cbColor)
        rospy.Subscriber("/camera/depth/image_raw", Image, self.cbDepth)

        if self.DYNAMIC_MAP:
            rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.cbGrid)

        rospy.Service("/telepresence/click", Click, self.cbClick)
        rospy.Service("/telepresence/rotate", Rotate, self.cbRotate)

        self.pubImage = rospy.Publisher("/telepresence/image", Image, queue_size=1)
        self.pubGrid = rospy.Publisher("telepresence/grid", Image, queue_size=1)
        self.pubGoal = rospy.Publisher("/porszilo/move_base/goal", PoseStamped, queue_size=5)
        self.pubCancelGoal = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)


        rate = rospy.Rate(self.FPS)
        try:
            while not rospy.is_shutdown():
                self.getTransform()
                self.createImage()
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
                self.tf_pose = self.tf_listener.lookupTransform('/camera_link', '/map', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
                rospy.logerr(ex)
                rospy.sleep(0.5)
        while True:
            try:
                self.tf_base_to_cam = self.tf_listener.lookupTransform('/camera_link', '/base_footprint', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
                rospy.logerr(ex)
                rospy.sleep(0.5)



    def cbClick(self, data):
        rospy.loginfo("Received click on {}".format((data.x, data.y)))

        pt = Point()
        pt.x = data.x * self.camera_info.width
        pt.y = data.y * self.camera_info.height
        
        res = ClickResponse()
        res.success = self.checkGrid(pt)
        if not self.checkGrid(pt):
            self.cancelGoal()
            return res
        
        try:
            pt3D = self.imgTransform.to3D(pt)
        except:
            raise Exception("")
        self.goal.pose.position = pt3D

        print(pt3D)

        dy = pt3D.y
        dx = pt3D.x
        yaw = atan(dy/dx)
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        self.goal.pose.orientation.x = quaternion[0]
        self.goal.pose.orientation.y = quaternion[1]
        self.goal.pose.orientation.z = quaternion[2]
        self.goal.pose.orientation.w = quaternion[3]

        self.pubGoal.publish(self.goal)

        return res

    def cbRotate(self, data):
        #TODO
        self.rotate = data


    def to2D(self, pt):
        v = np.array([pt.x, pt.y, 0, 1])
        v = self.rot_mat.dot(v)
        ret = Point()
        ret.x = v[0]
        ret.y = v[1]
        return ret

    def to3D(self, pt):
        v = np.array([pt.x, pt.y, pt.z])
        v = np.linalg.inv(self.rot_mat).dot(v)
        ret = Point()
        ret.x = v[0] - self.tf_pose[0][0]
        ret.y = v[1] - self.tf_pose[0][1]
        ret.z = v[2] - self.tf_pose[0][2]
        return ret



    def doRotate(self):
        if not self.rotate.on:
            return

        pos = (self.rotate.x, self.rotate.y)
        self.image.setflags(write=1)
        thisarrow = self.arrow_pic
        if self.rotate.left:
            thisarrow = cv2.flip(thisarrow, 1)
        self.image = self.overlayImage(self.image, thisarrow, pos)

        self.goal.pose.position.x = self.tf_pose[0][0]
        self.goal.pose.position.y = self.tf_pose[0][1]
        self.goal.pose.position.z = self.tf_pose[0][2]

        quaternion = (
            self.tf_pose[1][0],
            self.tf_pose[1][1],
            self.tf_pose[1][2],
            self.tf_pose[1][3])
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]
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
        #TODO
        pass


    def placeMarker(self):
        if not self.goal_on:
            return

        pt = Point()
        pt.x = self.goal.pose.position.x
        pt.y = self.goal.pose.position.y
        pt = self.imgTransform.to2D(pt)
        pos = (pt.x, pt.y-self.marker_pic.shape[1]/2)

        self.image.setflags(write=1)
        self.image = self.overlayImage(self.image, self.marker_pic, pos)


    def pubGridImage(self):
        return
        grid_img = np.array(self.grid.data) / 255.0
        grid_img = np.reshape(grid_img, (self.grid.info.height, self.grid.info.width)).astype('uint8')
        zeros = np.zeros([self.grid.info.height, self.grid.info.width, 4])
        zeros[:,:,0] = grid_img
        #  zeros[:,:,1] = grid_img
        #  zeros[:,:,2] = grid_img

        grid_img = cv2.cvtColor(zeros, cv2.COLOR_BGR2GRAY)
        #  grid_img = cv2.cvtColor(zeros, cv2.COLOR_GRAY2BGR)
        self.cvBridge.cv2_to_imgmsg(grid_img)
        self.pubGrid.publish(grid_img)


    def createImage(self):
        self.image = self.cv_color

        self.doRotate()
        self.placePoints()
        self.placeMarker()

        self.pubImage.publish(self.cvBridge.cv2_to_imgmsg(self.image, encoding="bgr8"))
        self.pubGridImage()


    def checkGrid(self, pt):# needs 2d points in screen coords. (geometry_msgs/Point)
        try:
            pt = self.imgTransform.to3D(pt)
        except:
            return False
        idx = self.camera_info.width * int(pt.y) + int(pt.x)
        data = self.grid.data[idx]
        return data < 50

    def cancelGoal(self):
        empty = GoalID()
        pubCancelGoal.publish(empty)

    def overlayImage(self, big, small, pos):
        for i in range(small.shape[0]):
            for j in range(small.shape[1]):
                if sum(small[i, j, :]) > 100:
                    try:
                        big[i+int(pos[0]), j+int(pos[1]), :] = small[i, j, :]
                    except:
                        pass
        return big


if __name__ == "__main__":
    t = Telepresence()
    t.run()
