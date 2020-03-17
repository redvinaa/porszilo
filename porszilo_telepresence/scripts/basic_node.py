#! /usr/bin/env python2.7
# -*- coding: utf-8 -*-

import sys
import rospy
from geometry_msgs.msg         import PoseStamped
from porszilo_telepresence.srv import ClickedPoint, ClickedPointResponse
from porszilo_telepresence.msg import CanvasSize
from sensor_msgs.msg           import Image
from nav_msgs.msg              import OccupancyGrid, Odometry
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
import matplotlib.pyplot as plt

class Telepresence():
    ## global variables:
        # color
        # depth
        # grid
        # odom
        # goal
    
    
    ## subscriber callbacks
    def cbColor(self, data):
        self.color = data
        self.cv_color = CvBridge().imgmsg_to_cv2(data, desired_encoding="passthrough")
    
    def cbDepth(self, data):
        self.depth = data
        self.cv_depth = CvBridge().imgmsg_to_cv2(data, desired_encoding="passthrough")
    
    def cbGrid(self, data):
        self.grid = data
    
    def cbOdom(self, data):
        self.odom = data

    def cbCanvasSize(self, data):
        self.canvas['x'] = data.width
        self.canvas['y'] = data.height
    
    
    ## loop
    def run(self):
        # 2D pic with the clickable areas marked
        # published to the browser

        #  self.getClickablePic() # fills self.clickable_pic, raises excetp on error
        try:
            self.getClickablePic() # fills self.clickable_pic, raises excetp on error
            self.clickable_pic = np.array(self.clickable_pic, dtype=np.dtype('uint8'))

        except rospy.ROSInternalException as err:
            rospy.logerr("ROSInternalException: {}".format(err))
            self.clickable_pic = self.cv_color

        self.imagePub.publish(CvBridge().cv2_to_imgmsg(self.clickable_pic, encoding="bgr8"))
        
    
    def getClickablePic(self):
        # calculate 2D pic with the clickable areas marked
	
	# az általam rajzolt térképen 1 pixel 1 cm a valóságban
	pos_y = int(round(self.odom.pose.pose.position.y/self.grid.info.resolution + self.grid.info.height/2)) # pozíciónk a térképen pixelben
	pos_x = int(round(self.odom.pose.pose.position.x/self.grid.info.resolution + self.grid.info.width/2))
	h = int(round(self.odom.pose.pose.position.z/self.grid.info.resolution))+5 # a kamer optikai tengelyének magassága a földtől pixel egységgé alakítva
	view_angle_v = self.view_angle_v  # kamera vertikális látószöge fokban
	view_angle_h = self.view_angle_h   # kamera horizontális látószöge fokban

	qx=self.odom.pose.pose.orientation.x
	qy=self.odom.pose.pose.orientation.y
	qz=self.odom.pose.pose.orientation.z
	qw=self.odom.pose.pose.orientation.w
	siny_cosp = 2 * (qw * qz + qx * qy)
	cosy_sinp = 1 - 2 * (qy * qy + qz * qz)
	yaw = math.atan2(siny_cosp, cosy_sinp)

	deg = round(90 - yaw * 180 / math.pi)
        # hány fokkal kell elforgatnunk a térépet hogy a robot nézési iránya függőlegesen legyen

        orig_map = np.array(self.grid.data)  # beolvassuk a térképet és a látott képet
	orig_map = np.where(orig_map==-1, 0, 255)
	
	orig_map = np.reshape(orig_map,(self.grid.info.height, self.grid.info.width)).astype('uint8')
	orig_map = cv2.flip(orig_map, 0)

	cam = self.cv_color

	cam_gray = cv2.cvtColor(cam, cv2.COLOR_BGR2GRAY)

	map_cols = orig_map.shape[1]
	map_rows = orig_map.shape[0]

	cam_cols = cam.shape[1]
	cam_rows = cam.shape[0]

	map_diag = int(round(1.2 * (math.sqrt(math.pow(map_cols, 2) + math.pow(map_rows, 2)))))  # egy nagy térkép amibe beletéve a kicsit szabadon el tudjuk forgatni
	big_map = np.zeros((map_diag, map_diag), 'uint8')

	shift_cols = int(round((map_diag - map_cols) / 2))  # beletesszük a közepére a térképet
	shift_rows = int(round((map_diag - map_rows) / 2))
	big_map[shift_rows:(map_rows + shift_rows), shift_cols:(map_cols + shift_cols)] = orig_map

	pos_y_big = shift_rows + pos_y  # átszámítva a koordinátkat a nagy térképre
	pos_x_big = shift_cols + pos_x

	M_rot = cv2.getRotationMatrix2D((map_diag / 2, map_diag / 2), deg, 1)  # elforgatjuk hogy függőlegesen fölfelé legyen a robot nézési iránya
	map_rot = cv2.warpAffine(big_map, M_rot, (map_diag, map_diag))

	# Hol vagyunk a forgatás után, a nagy térképen?
	big_map_half = int(round(map_diag / 2))
	rad = deg * math.pi / 180
	pos_x_rot = int((pos_x_big - big_map_half) * math.cos(rad) + (pos_y_big - big_map_half) * math.sin(rad) + big_map_half)
	pos_y_rot = int(-(pos_x_big - big_map_half) * math.sin(rad) + (pos_y_big - big_map_half) * math.cos(rad) + big_map_half)

	# keressük meg a perspektívikusan torzítandó képrészlet 4 sarkát
	# ez itt puszta koordinátageometria kiegyszerűsítve, átláthatatlanul

	y3 = pos_y_rot - int(round(h * math.tan((90 - view_angle_v / 2) * math.pi / 180)))  # 324

        try:
            y4 = np.where(big_map == 255)[0][0]  # 14
        except:
            raise rospy.ROSInternalException("received occupancy grid is probably empty, retrying \n(original error is: {})".format(sys.exc_info()))

	m = math.tan((90 - view_angle_h / 2) * math.pi / 180)  # 1.28
	b1 = pos_y_rot + m * pos_x_rot  # 711.6
	b2 = pos_y_rot - m * pos_x_rot  # 20.4

	p1x = int(round((y3 - b2) / m))
	p1y = int(round(m * p1x + b2))

	p2x = int(round((y4 - b2) / m))
	p2y = int(round(m * p2x + b2))

	p3x = int(round((-y4 + b1) / m))
	p3y = int(round(-m * p3x + b1))

	p4x = int(round((-y3 + b1) / m))
	p4y = int(round(-m * p4x + b1))

	# mennyire kell összenyomni a képet
	beta = math.atan((pos_y_rot - y4) / h) * 180 / math.pi - 90 + view_angle_v / 2
	ratio = beta / view_angle_v

	output_rows = int(round(cam_rows * ratio))
	output_cols = int(round(cam_cols))

	pts1 = np.float32([[p2x, p2y], [p3x, p3y], [p1x, p1y], [p4x, p4y]])
	pts2 = np.float32([[0, 0], [output_cols, 0], [0, output_rows], [output_cols, output_rows]])
	M_persp = cv2.getPerspectiveTransform(pts1, pts2)
	output = cv2.warpPerspective(map_rot, M_persp, (output_cols, output_rows))
	output = output

	output2 = np.array([cam_gray], dtype=int)
	output2[0, cam_rows - output_rows - 1:-1, :] = output
	output2 = np.where(output2 < 80, cam_gray, output2)

	final2 = np.zeros((cam_rows, cam_cols, 3))
	final2[:, :, 0] = cam_gray
	final2[:, :, 1] = output2
	final2[:, :, 2] = cam_gray

	
	#cv2.imshow("ablak", final2/255)
	#cv2.waitKey(1)
	self.clickable_pic = final2


    def clickedTo3D(self):

        # this needs to return bool: success of click service call
        # also, this should fill up the self.goal field

        # the reason some values return nan, is because
        # the given values are outside of the field-of.view
        # of the rgbd camera (usually too close or too far from it)

        self.clicked_depth = self.cv_depth[self.point_y, self.point_x]


        rospy.loginfo("----------------")
        rospy.loginfo("Clicked depth: {}".format(self.clicked_depth))

        if np.isnan(self.clicked_depth):
            return False # success of service call

        # quaternions to yaw
	qx=self.odom.pose.pose.orientation.x
	qy=self.odom.pose.pose.orientation.y
	qz=self.odom.pose.pose.orientation.z
	qw=self.odom.pose.pose.orientation.w
	siny_cosp = 2 * (qw * qz + qx * qy)
	cosy_sinp = 1 - 2 * (qy * qy + qz * qz)
	yaw = math.atan2(siny_cosp, cosy_sinp)

        # self.goal is of type geometry_msgs/PoseStamped
        self.goal.header.stamp = rospy.Time.now()

        assert(self.odom.header.frame_id and self.odom.child_frame_id)
        self.goal.header.frame_id = self.odom.header.frame_id

        self.goal.pose.position.x = self.odom.pose.pose.position.x + math.cos(yaw) * self.clicked_depth
        self.goal.pose.position.y = self.odom.pose.pose.position.y + math.sin(yaw) * self.clicked_depth
        #  self.goal.pose.position.z = ...
        #TODO
        # itt minden default 0, a quaternion nem lehet viszont nulla
        # that's illegal
        self.goal.pose.orientation.w = 1 # hasraütés szerűen

        return True

        
    def cbClickedPoint(self, req):
        # ebbol megvan a pixel helye: req.x, req.y
        # ha click-elnek, itt dolgozzuk fel es kuldjuk el
        # (pixel -> 3D trafo, aztan publishelni)
        # innen nem kell beszelni a web klienssel

        self.point_x = int(req.x * self.cv_depth.shape[1] / self.canvas['x']) # width
        self.point_y = int(req.y * self.cv_depth.shape[0] / self.canvas['y']) # height

        if self.point_x > self.cv_depth.shape[1]:
            self.point_x = self.cv_depth.shape[1] - 1
        if self.point_y > self.cv_depth.shape[0]:
            self.point_y = self.cv_depth.shape[0] - 1

        res = ClickedPointResponse()
        res.success = self.clickedTo3D()

        self.goalPub.publish(self.goal)

        return res
    
    def main(self):
        rospy.init_node('telepresence')
    
        self.goal = PoseStamped()
        self.canvas = {}
        self.canvas['x']  = rospy.get_param("canvas_width", 700)
        self.canvas['y']  = rospy.get_param("canvas_height", 500)
        freq              = rospy.get_param("frequency", 10) # default 10 Hz
	self.view_angle_v = rospy.get_param("view_angle_v", 50)
	self.view_angle_h = rospy.get_param("view_angle_h", 76)
    
        rospy.loginfo('Waiting for messages to arrive...')
        try:
            # elsőre megvárjuk hogy minden legalább egyszer megérkezzen, így minden
            # define-olva lesz és mindenben lesz adat
            self.cbColor(rospy.wait_for_message("/camera/color/image_raw", Image))
            self.cbDepth(rospy.wait_for_message("/camera/depth/image_raw", Image))
            self.cbGrid(rospy.wait_for_message("/move_base/global_costmap/costmap", OccupancyGrid))
            self.cbOdom(rospy.wait_for_message("/odometry/filtered", Odometry))

        except rospy.ROSInterruptException:
            rospy.loginfo("Telepresence node is being shut down")
            quit()

        try_again_after = 2
        rate = rospy.Rate(1.0/try_again_after)
        while self.grid.data[1:] == self.grid.data[:-1]: # while all elements are the same (means map is empty)
            rospy.logerr("Received occupancy grid is empty, trying again in {} seconds".format(try_again_after))
            rate.sleep()
            self.cbGrid(rospy.wait_for_message("/move_base/global_costmap/costmap", OccupancyGrid))

        rospy.Subscriber("/camera/color/image_raw", Image, self.cbColor)
        rospy.Subscriber("/camera/depth/image_raw", Image, self.cbDepth)
        rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.cbGrid)
        rospy.Subscriber("/odometry/filtered", Odometry, self.cbOdom)
        rospy.Subscriber("/telepresence/canvas_size", CanvasSize, self.cbCanvasSize)
    
        rospy.loginfo('All messages have arrived at least once, starting publishers')
        self.goalPub  = rospy.Publisher("/porszilo/move_base/goal", PoseStamped, queue_size=5)
        self.imagePub = rospy.Publisher("/telepresence/image", Image, queue_size=5)

        # think of this as a remote function call, called by the  JS code in the browser
        self.srv_clicked_point = rospy.Service("/telepresence/clicked_point", ClickedPoint, self.cbClickedPoint)
    
        rate = rospy.Rate(freq)
        try:
            while not rospy.is_shutdown():
                self.run()# <- this is being looped
                rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("Telepresence node is being shut down")
            quit()


if __name__ == "__main__":

    tp = Telepresence()
    tp.main()
