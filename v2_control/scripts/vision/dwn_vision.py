#!/usr/bin/python

import argparse
import rospy
import numpy as np
import cv2
from threading import Thread
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

## Global variables
bridge = CvBridge() # CVBridge object

"""
Coverts ROS msg into OpenCV image matrix, calibrates the image, and then
calls function to further process the image.
"""
class Dwn_imgproc (Thread):
	def __init__(self, img_topic, img_w, img_h, cam_mtx, dist_coef):
		Thread.__init__(self)
		# initialize the thread
		self.stop_thread = False

		# subscribing to ROS topic
		self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)

		# obtaining image parameters
		self.img_w = img_w
		self.img_h = img_h
		self.cam_mtx = cam_mtx
		self.dist_coef = dist_coef
		
		# obtaining the camera matrix
		self.newCamMtx, _ = cv2.getOptimalNewCameraMatrix(self.cam_mtx, self.dist_coef,
									(self.img_w,self.img_h), 1,
									 (self.img_w,self.img_h))
	
	def run(self):
		while not self.stop_thread:
			pass
		print("Ended dwn img processing.")

	def callback(self, data):
		# converting ROS msg to CV img
		try:
			cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		cv2.imshow('Down Vision', cv2.undistort(cv_image, self.cam_mtx, self.dist_coef, None, self.newCamMtx))
		cv2.waitKey(1)

if __name__ == '__main__':
	# cli argument parser
	parser = argparse.ArgumentParser(description="Generic rospy node to handle AUV_v2 down vision",
		epilog="Developed by AUV Society, IIITDM")

	# down camera args
	parser.add_argument("--d_width", type=int, help="Width of the image from down camera | Default: 640",
						default=640)
	parser.add_argument("--d_height", type=int, help="Height of the image from down camera | Default: 480", 
						default=480)
	
	# parse the arguments
	arg, _ = parser.parse_known_args()
	args = vars(arg)

	rospy.init_node('auv_dwn_vision')	# initialising rospy node

	# Define Camera Matrix
	cam_mtx =  np.array([[421.375597, 0, 319.170565],
					 [0, 421.314788, 239.644714],
					 [0, 0, 1]])

	# Define distortion coefficients
	dist_coef = np.array([-0.000233, -0.000328, 0.000116, -0.000326, 0.000000])

	# define image calibration threads
	th_dwn = Dwn_imgproc("/auv_v2/d_cam/image_raw",
							args["d_width"], args["d_height"], cam_mtx, dist_coef)
	th_dwn.daemon = True

	th_dwn.start()

	print("Started processing front and down cameras' output...")

	try:
		rospy.spin()
	except:
		# sending stop command
		th_dwn.stop_thread = True
		th_dwn.join()
	finally:
		print("Closing AUV Down vision.")