import argparse
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

import tf
import tf2_ros
import rospy
import intera_interface
from sensor_msgs.msg import *
from pyquaternion import Quaternion
from find_path import solve
from binarization import binarization
from PIL import Image
from image_processing import find
from intera_interface import gripper as robot_gripper



image = None
extra_width = 20

#meters

#top_left [0.8404, 0.1972, -0.0082]
#bottom_left [0.3341, 0.1627. -0.0090]
#bottom_right [0.4429, -0.5124, -0.0172]

delta = 0.1
maze_hight = 0.5 + delta
maze_width = 0.7 + delta
tag_position = np.array([45, 80])

start = tag_position - np.array([0, 70])
end = tag_position - np.array([5,5])

#space to pixel ratio
ratio = 100

# h, w = maze_hight * ratio, maze_width * ratio
h, w = 140, 190
cam = "right_hand_camera"

# def show_image_callback(img_data.args):
# 	bridge = CvBridge()
#	 try:
#		 cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")
#	 nonlocal image 
#	 image = cv_image.threashold()
#	 im2 = np.copy(image)
#	 for i in len(image):
#	 	for j in len(image[0]):

def intrinsic_matrix_callback(camera_info_msg):
	global K
	K = np.reshape(np.array(camera_info_msg.K), (3,3))


def get_image_callback(camera_msg):
	global img 
	bridge = CvBridge()
	try:
		img = bridge.imgmsg_to_cv2(camera_msg, "bgr8")
	except CvBridgeError, err:
		rospy.logerr(err)
		return

def get_intrinsic_matrix():
	# rospy.init_node('intrinsic_listener', anonymous=True)
	rospy.Subscriber("/io/internal_camera/right_hand_camera/camera_info", sensor_msgs.msg.CameraInfo, intrinsic_matrix_callback)
	# rospy.spin()

def get_image():
	rospy.Subscriber("/io/internal_camera/right_hand_camera/image_raw", sensor_msgs.msg.Image, get_image_callback)

def vision():
	start_camera()
	get_intrinsic_matrix()
	while True:
		try:
			get_image()
			break
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print("f")
			pass

	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	"""Camera Display Example
	Cognex Hand Camera Ranges
		- exposure: [0.01-100]
		- gain: [0-255]
	Head Camera Ranges:
		- exposure: [0-100], -1 for auto-exposure
		- gain: [0-79], -1 for auto-gain
	"""

	# plane : the work plane wrt the ar tag
	top_left_plane = np.array([-0.85, 0.6, 0, 1])
	top_right_plane = np.array([0.1, 0.6, 0, 1])
	bottom_left_plane = np.array([-0.85, -0.1, 0, 1])
	bottom_right_plane = np.array([0.1, -0.1, 0, 1])
	tag_plane = np.array([0, 0, 0, 1])
	r = rospy.Rate(10)
	now = rospy.Time.now()
	# while not rospy.is_shutdown():
	# 	try:
	# 		trans1 = tfBuffer.lookup_transform('reference/head_camera', 'head_camera', rospy.Time())
	# 		break
	# 	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
	# 		if tf2_ros.LookupException:
	# 			print("lookup")
	# 		elif tf2_ros.ConnectivityException:
	# 			print("connect")
	# 		elif tf2_ros.ExtrapolationException:
	# 			print("extra")
	# 		# print("not found")
	# 		pass
	# 	r.sleep()
	# print(trans1.transform)
	while not rospy.is_shutdown():
		try:
			trans1 = tfBuffer.lookup_transform('right_hand_camera', 'ar_marker_0', rospy.Time())
			break
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			if tf2_ros.LookupException:
				print("lookup")
			elif tf2_ros.ConnectivityException:
				print("connect")
			elif tf2_ros.ExtrapolationException:
				print("extra")
			# print("not found")
			pass
		r.sleep()
	# t = tf.TransformerROS(True, rospy.Duration(10.0))
	# t.setTransform(trans1)
	# trans1 = t.asMatrix('head_camera', 'ar_marker_11')
	trans1 = as_transformation_matrix(trans1.transform)
	top_left_space = np.dot(trans1, top_left_plane)
	top_right_space = np.dot(trans1, top_right_plane)
	bottom_left_space = np.dot(trans1, bottom_left_plane)
	bottom_right_space = np.dot(trans1, bottom_right_plane)
	tag_space = np.dot(trans1, tag_plane)
	

	top_left_pixel = np.dot(K, top_left_space[:3])
	top_left_pixel = (top_left_pixel[:2]/top_left_pixel[2]).astype(int)
	top_right_pixel = np.dot(K , top_right_space[:3])
	top_right_pixel = (top_right_pixel[:2]/top_right_pixel[2]).astype(int)
	bottom_left_pixel = np.dot(K , bottom_left_space[:3])
	bottom_left_pixel = (bottom_left_pixel[:2]/bottom_left_pixel[2]).astype(int)
	bottom_right_pixel = np.dot(K , bottom_right_space[:3])
	bottom_right_pixel = (bottom_right_pixel[:2]/bottom_right_pixel[2]).astype(int)
	tag_pixel = np.dot(K , tag_space[:3])
	tag_pixel = (tag_pixel[:2]/tag_pixel[2]).astype(int)

	start_pixel = np.array([45, 30])
	end_pixel = np.array([100, 150]) 

	pixel_list = np.array([top_left_pixel[::-1], top_right_pixel[::-1], bottom_left_pixel[::-1], bottom_right_pixel[::-1]])
	print(tag_pixel)
	print(pixel_list)
	ideal_list = np.array([(0,0), (0, w), (h, 0), (h, w)])

	# M, _ = cv2.findHomography(pixel_list, ideal_list)
	# out = np.zeros(h, w)
	waypoints = find(img, pixel_list, ideal_list, start_pixel, end_pixel)

	#M_inv, _ = cv2.findHomography(ideal_list, pixel_list)
	tag = np.array([120,170])
	waypoints_abstract = []
	for point in waypoints:
		x = float(point[1]-tag[1])/200
		y = float(tag[0]-point[0])/200
		point = np.array([x, y, 0, 1])
		waypoints_abstract.append(point)
	print('points\n')
	print(waypoints_abstract)
	return waypoints_abstract



	# inverse_M = np.linalg.inv(M)
	# inverse_trans = np.linalg.inv(trans)

	# waypoints_image = []
	# for point in waypoints:
	# 	image_point = np.append(inverse_M@point, [0], axis = 0)
	# 	waypoints_image.append(inverse_trans@image_point)


	#	M2, _ = cv2.findHomography([top_left_ideal, top_right_ideal, bottom_left_ideal, bottom_right_ideal],[top_left_space, top_right_space, bottom_left_space, bottom_right_space])

def as_transformation_matrix(trans):
	"""Convert geomety_msgs.msg.TransformStamped to Transformation matrix.
	Args:
		trans (geomety_msgs.msg.TransformStamped): transform to be converted.
	Returns:
		numpy.array: transformation matrix generated.
	"""
	mat = Quaternion(
		trans.rotation.w,
		trans.rotation.x,
		trans.rotation.y,
		trans.rotation.z)
	mat = mat.transformation_matrix
	mat[0][3] = trans.translation.x
	mat[1][3] = trans.translation.y
	mat[2][3] = trans.translation.z
	return mat
	
def start_camera():
	# right_gripper = robot_gripper.Gripper('right')
	# right_gripper.close()
	# rp = intera_interface.RobotParams()
	# valid_cameras = rp.get_camera_names()
	# if not valid_cameras:
	#	 rp.log_message(("Cannot detect any camera_config"
	#		 " parameters on this robot. Exiting."), "ERROR")
	#	 return
	print("Initializing node... ")
	# rospy.init_node('camera_display', anonymous=True)
	cameras = intera_interface.Cameras()
	# if not cameras.verify_camera_exists(cam):
	#	 rospy.logerr("Could not detect the specified camera, exiting the example.")
	#	 return
	rospy.loginfo("Opening camera '{0}'...".format(cam))
	cameras.start_streaming(cam)
	# cameras.set_callback(cam, show_image_callback, callback_args=(use_canny_edge, cam))

	# optionally set gain and exposure parameters

	# def clean_shutdown():
	#	 print("Shutting down camera_display node.")
	#	 cv2.destroyAllWindows()

	# rospy.on_shutdown(clean_shutdown)
	# rospy.loginfo("Camera_display node running. Ctrl-c to quit")
	# rospy.spin()


if __name__ == '__main__':
	vision()