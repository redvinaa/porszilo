#! /usr/bin/env python2.7
# -*- coding: utf-8 -*-

import sys
import rospy
from geometry_msgs.msg         import PoseStamped, Twist
from porszilo_telepresence.srv import ClickedPoint, ClickedPointResponse
from porszilo_telepresence.msg import CanvasSize
from std_msgs.msg              import String
from actionlib_msgs.msg        import GoalStatusArray
from sensor_msgs.msg           import Image
from nav_msgs.msg              import OccupancyGrid
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
import matplotlib.pyplot as plt
import tf
import rospkg
import os


class Telepresence():
	## global variables:
		# color
		# depth
		# grid
		# tf_pose
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

	def cbCanvasSize(self, data):
		self.canvas['x'] = data.width
		self.canvas['y'] = data.height

	def cbGoalStatus(self, data):
		self.move_base_working = False
		try:
			if data.status_list[-1].status != 3:
				self.move_base_working = True
		except:
			pass # False

	def cbTurnButtons(self, data):
		# btn-left-md
		# btn-left-mu
		# btn-right-md
		# btn-right-mu

		ex = rospy.ROSInternalException("invalid data received on '/telepresence/turn_buttons'")
		if "left" in data.data:
			if "md" in data.data:
				self.btn_left = "md"
			elif "mu" in data.data:
				self.btn_left = "mu"
			else:
				raise ex
		elif "right" in data.data:
			if "md" in data.data:
				self.btn_right = "md"
			elif "mu" in data.data:
				self.btn_right = "mu"
			else:
				raise ex
		else:
			raise ex


	## loop
	def run(self):
		# 2D pic with the clickable areas marked
		# published to the browser

		try:
			self.tf_pose = self.listener.lookupTransform('/camera_link', '/map', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
			print(ex)

		#  self.getClickablePic() # fills self.clickable_pic, raises excetp on error
		try:
			self.getClickablePic() # fills self.clickable_pic, raises excetp on error
			self.clickable_pic = np.array(self.clickable_pic, dtype=np.dtype('uint8'))

		except rospy.ROSInternalException as err:
			rospy.logerr("ROSInternalException: {}".format(err))
			self.clickable_pic = self.cv_color

		self.imagePub.publish(CvBridge().cv2_to_imgmsg(self.clickable_pic, encoding="bgr8"))

		if self.btn_left == "md":
			self.rot_msg.angular.z = self.max_rot_vel
		elif self.btn_right == "md":
			self.rot_msg.angular.z = -self.max_rot_vel
		else:
			self.rot_msg.angular.z = 0
		self.pubTurnButtons.publish(self.rot_msg)


	def getClickablePic(self):
		# calculate 2D pic with the clickable areas marked

	self.pos_y = int(round(self.tf_pose[0][1]/self.grid.info.resolution + self.grid.info.height/2)) # pozíciónk a térképen pixelben
	self.pos_x = int(round(self.tf_pose[0][0]/self.grid.info.resolution + self.grid.info.width/2))
	h = self.tf_pose[0][2]/self.grid.info.resolution + 9.1
	#  h = self.tf_pose[0][2]/self.grid.info.resolution + 2.1 # TODO


	view_angle_v = self.view_angle_v  # kamera vertikális látószöge fokban
	view_angle_h = self.view_angle_h   # kamera horizontális látószöge fokban

		q = (\
				self.tf_pose[1][0],\
				self.tf_pose[1][1],\
				self.tf_pose[1][2],\
				self.tf_pose[1][3])
		euler = tf.transformations.euler_from_quaternion(q)

	yaw = euler[2]

	deg = int(round((90 - yaw * 180 / math.pi) - 0.5))
	deg = int(round((90 + yaw * 180 / math.pi) - 0.5))
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

	self.shift_cols = int(round((map_diag - map_cols) / 2))  # beletesszük a közepére a térképet
	self.shift_rows = int(round((map_diag - map_rows) / 2))
	big_map[self.shift_rows:(map_rows + self.shift_rows), self.shift_cols:(map_cols + self.shift_cols)] = orig_map


	pos_y_big = self.shift_rows + self.pos_y  # átszámítva a koordinátkat a nagy térképre
	pos_x_big = self.shift_cols + self.pos_x

	M_rot = cv2.getRotationMatrix2D((map_diag / 2, map_diag / 2), deg, 1)  # elforgatjuk hogy függőlegesen fölfelé legyen a robot nézési iránya
	map_rot = cv2.warpAffine(big_map, M_rot, (map_diag, map_diag))

	# Hol vagyunk a forgatás után, a nagy térképen?
	self.big_map_half = int(round(map_diag / 2))
	self.rad = deg * math.pi / 180
	pos_x_rot = int((pos_x_big - self.big_map_half) * math.cos(self.rad) + (pos_y_big - self.big_map_half) * math.sin(self.rad) + self.big_map_half)
	pos_y_rot = int(-(pos_x_big - self.big_map_half) * math.sin(self.rad) + (pos_y_big - self.big_map_half) * math.cos(self.rad) + self.big_map_half)

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

	#print(p1x, p1y, '  ', p2x, p2y, '  ', p3x, p3y, '  ', p4x, p4y)


	# mennyire kell összenyomni a képet
	beta = math.atan((pos_y_rot - y4) / h) * 180 / math.pi - 90 + view_angle_v / 2
	self.ratio = beta / view_angle_v

	output_rows = int(round(cam_rows * self.ratio))
	output_cols = int(round(cam_cols))

	pts1 = np.float32([[p2x, p2y], [p3x, p3y], [p1x, p1y], [p4x, p4y]])
	pts2 = np.float32([[0, 0], [output_cols, 0], [0, output_rows], [output_cols, output_rows]])


	self.M_persp = cv2.getPerspectiveTransform(pts1, pts2)
	output = cv2.warpPerspective(map_rot, self.M_persp, (output_cols, output_rows))
	output = np.where(output < 80, 0, 255)

	output2 = np.array([cam_gray], dtype=int)
	output2[0, cam_rows - output_rows - 1:-1, :] = output
	output2 = np.where(output2 < 80, cam_gray, output2)


		cam_copy = cam.copy()

	cam_copy[:, :, 1] = np.where(output2 < 80, cam[:, :, 1], output2)

	self.clickable_pic = cam_copy

	if self.drawGoal and self.move_base_working:# <- bool
		self.placeMarker()

	def placeMarker(self):

		#goal = np.array([[[self.goal.pose.position.x, self.goal.pose.position.y]]])

	goal_y = int(-self.goal.pose.position.y/self.grid.info.resolution + self.grid.info.height/2)
	goal_x = int((self.goal.pose.position.x)/self.grid.info.resolution + self.grid.info.width/2)

	goal_y_big = self.shift_rows + goal_y  
	goal_x_big = self.shift_cols + goal_x

	goal_y_rot = int(-(goal_x_big - self.big_map_half) * math.sin(self.rad) + (goal_y_big - self.big_map_half) * math.cos(self.rad) + self.big_map_half)
	goal_x_rot = int((goal_x_big - self.big_map_half) * math.cos(self.rad) + (goal_y_big - 	self.big_map_half) * math.sin(self.rad) + self.big_map_half)

	#print(goal_x_rot, goal_y_rot)


	goal = np.array([[[goal_x_rot, goal_y_rot]]], dtype=np.float32)
	goal = cv2.perspectiveTransform(goal, self.M_persp);
	goal[0][0][0] = round(goal[0][0][0])
	goal[0][0][1] = round(goal[0][0][1] + (1 - self.ratio) * self.clickable_pic.shape[0])	

	#print(self.clickable_pic.shape)
	#  print("marker helye: ", goal[0][0])
	#  print("kattintasbol szamolt hely: ", self.goal.pose.position.x, self.goal.pose.position.y)

	#dist = (math.pow((goal_y  - self.pos_y), 2) + math.pow((goal_x - self.pos_y), 2)) * 0.001

	scale = (self.clickable_pic.shape[0] / self.marker.shape[0]) * 0.1 #/ dist * 10
	new_size_x = int(self.marker.shape[0] * scale)
	new_size_y = int(self.marker.shape[1] * scale)
	#print(scale, new_size_x, new_size_y)
	self.marker = cv2.resize(self.marker, (new_size_x, new_size_y))

	place_y = int(abs(goal[0][0][1] - new_size_y * 0.87))
	place_x = int(abs(goal[0][0][0] - new_size_x / 2))
	#print(place_x, place_y)

#--------------------------------------------------------------------------------------------#
	diff_y = self.clickable_pic[place_y : place_y + new_size_y, place_x  : place_x + new_size_x, :].shape[0]

	diff_x = self.clickable_pic[place_y : place_y + new_size_y, place_x  : place_x + new_size_x, :].shape[1]


	if  diff_x > self.marker.shape[1]*0.5:
		if diff_y > self.marker.shape[0]*0.87:
			marker2 =self.marker[0:diff_y, 0:diff_x, :]
			self.clickable_pic[place_y : place_y + new_size_y, place_x  : place_x + new_size_x, :] = cv2.bitwise_and(self.clickable_pic[place_y : place_y + new_size_y, place_x  : place_x + new_size_x, :], marker2)

	##Ide else-be akár mehetne valami hibaüzi a böngészős felületre, hogy nem a képen van a marker
#-----------------------------------------------------------------------------------------#	 


	def clickedTo3D(self):

		# this returns bool: success of click service call

		if self.point_pix_y <= self.canvas['y']/2:
			self.drawGoal = False
			return False

		#TODO
		height = self.tf_pose[0][2]
		#  height = self.tf_pose[0][2] * 1.0# <- empirikus
		view_angle_v_rad = self.view_angle_v * math.pi / 180
		view_angle_h_rad = self.view_angle_h * math.pi / 180

		tan_ang_v = (self.point_pix_y - self.canvas['y']/2) * \
				math.tan(view_angle_v_rad/2) / (self.canvas['y']/2)
		tan_ang_h = - (self.point_pix_x - self.canvas['x']/2) * \
				math.tan(view_angle_h_rad/2) / (self.canvas['x']/2)
		dist = height / tan_ang_v
		ang_h = math.atan(tan_ang_h)

		if dist > 10:
			self.drawGoal = False
			return False

		q = (\
				self.tf_pose[1][0],\
				self.tf_pose[1][1],\
				self.tf_pose[1][2],\
				self.tf_pose[1][3])
		euler = tf.transformations.euler_from_quaternion(q)

	yaw = euler[2]
		ang_h += yaw

	pose_x = self.tf_pose[0][0] + math.cos(ang_h) * dist
	pose_y = self.tf_pose[0][1] + math.sin(ang_h) * dist

		# self.goal is of type geometry_msgs/PoseStamped
		self.goal.header.stamp = rospy.Time.now()
		self.goal.header.frame_id = "map"

		self.goal.pose.position.x = pose_x
		self.goal.pose.position.y = pose_y
		self.goal.pose.position.z = 0

		quaternion = tf.transformations.quaternion_from_euler(0, 0, ang_h)
		self.goal.pose.orientation.x = quaternion[0]
		self.goal.pose.orientation.y = quaternion[1]
		self.goal.pose.orientation.z = quaternion[2]
		self.goal.pose.orientation.w = quaternion[3]

	self.drawGoal = True
		return True


	def cbClickedPoint(self, req):
		# ebbol megvan a pixel helye: req.x, req.y
		# ha click-elnek, itt dolgozzuk fel es kuldjuk el
		# (pixel -> 3D trafo, aztan publishelni)
		# innen nem kell beszelni a web klienssel

		self.point_x = int(req.x * self.cv_depth.shape[1] / self.canvas['x']) # width
		self.point_y = int(req.y * self.cv_depth.shape[0] / self.canvas['y']) # height
		self.point_pix_x = req.x
		self.point_pix_y = req.y

		if self.point_x > self.cv_depth.shape[1]:
			self.point_x = self.cv_depth.shape[1] - 1
		if self.point_y > self.cv_depth.shape[0]:
			self.point_y = self.cv_depth.shape[0] - 1

		res = ClickedPointResponse()
		res.success = self.clickedTo3D()

		if res.success:
			self.goalPub.publish(self.goal)

		return res

	def main(self):
		rospy.init_node('telepresence')

	rospack = rospkg.RosPack()
	pkg_path = rospack.get_path('porszilo_telepresence')
	pkg_path = os.path.join(pkg_path, "marker.png")
	self.marker = cv2.imread(pkg_path)


		self.goal = PoseStamped()
		self.canvas = {}
		self.canvas['x']  = rospy.get_param("canvas_width", 700)
		self.canvas['y']  = rospy.get_param("canvas_height", 500)
		freq              = rospy.get_param("frequency", 10) # default 10 Hz
	self.view_angle_v = rospy.get_param("view_angle_v", 50)
	self.view_angle_h = rospy.get_param("view_angle_h", 76)
		self.max_rot_vel  = rospy.get_param("max_rot_vel", 0.5)
		self.dynamic_map  = rospy.get_param("dynamic_map", False)

		self.listener          = tf.TransformListener()
	self.drawGoal          = False
		self.btn_left          = ''
		self.btn_right         = ''
		self.rot_msg           = Twist()
		self.move_base_working = False # move base is making the robot go to the goal (has not arrived)

		rospy.logwarn('Waiting for messages to arrive...')
		try:
			# elsőre megvárjuk hogy minden legalább egyszer megérkezzen, így minden
			# define-olva lesz és mindenben lesz adat
			self.cbColor(rospy.wait_for_message("/camera/color/image_raw", Image))
			self.cbDepth(rospy.wait_for_message("/camera/depth/image_raw", Image))
			self.cbGrid(rospy.wait_for_message("/move_base/global_costmap/costmap", OccupancyGrid))
			while True:
				try:
					self.tf_pose = self.listener.lookupTransform('/camera_link', '/map', rospy.Time(0))
					break
				except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
					print(ex)
					rospy.sleep(0.5)

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
		rospy.Subscriber("/telepresence/canvas_size", CanvasSize, self.cbCanvasSize)
		rospy.Subscriber("/telepresence/turn_buttons", String, self.cbTurnButtons)
		rospy.Subscriber("/move_base/status", GoalStatusArray, self.cbGoalStatus)
		if self.dynamic_map:
			rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.cbGrid)

		rospy.logwarn('All messages have arrived at least once, starting publishers')
		self.goalPub        = rospy.Publisher("/porszilo/move_base/goal", PoseStamped, queue_size=5)
		self.imagePub       = rospy.Publisher("/telepresence/image", Image, queue_size=5)
		self.pubTurnButtons = rospy.Publisher("/cmd_vel", Twist, queue_size=5)

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
