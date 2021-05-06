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
Coverts ROS msg into OpenCV image matrix, calibrates the image, publishes it.
"""
class FrntImgCalib (Thread):
	def __init__(self, img_topic, img_w, img_h, cam_mtx, dist_coef):
		# initialize the thread
		Thread.__init__(self)
		
		self.stop_thread = False

		# obtaining image parameters
		self.img_w = img_w
		self.img_h = img_h
		self.cam_mtx = cam_mtx
		self.dist_coef = dist_coef
		
		# obtaining the camera matrix
		self.newCamMtx, _ = cv2.getOptimalNewCameraMatrix(self.cam_mtx, self.dist_coef,
									(self.img_w,self.img_h), 1,
									 (self.img_w,self.img_h))
		
		# subscribing to ROS topic
		self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)
		
		# publishing to ROS topic
		self.image_pub = rospy.Publisher("/auv_v2/f_cam/image_calib", Image, queue_size=100)

	def run(self):
		while not self.stop_thread:
			pass
		print("Ended frnt img processing.")

	def callback(self, data):
		# converting ROS msg to CV img
		try:
			cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
			self.image_pub.publish(bridge.cv2_to_imgmsg(
				cv2.undistort(
					cv_image, self.cam_mtx, self.dist_coef, None, self.newCamMtx), "bgr8"))
		except CvBridgeError as e:
			print(e)

if __name__ == '__main__':
	# cli argument parser
	parser = argparse.ArgumentParser(description="Generic rospy node to handle AUV_v2 front vision",
		epilog="Developed by AUV Society, IIITDM")

	# front camera args
	parser.add_argument("--f_width", type=int, help="Width of the image from front camera | Default: 640",
						default=640)
	parser.add_argument("--f_height", type=int, help="Height of the image from front camera | Default: 480", 
						default=480)
	
	# parse the arguments
	arg, _ = parser.parse_known_args()
	args = vars(arg)

	rospy.init_node('auv_frnt_vision')	# initialising rospy node

	# Define Camera Matrix
	cam_mtx =  np.array([[421.375597, 0, 319.170565],
					 [0, 421.314788, 239.644714],
					 [0, 0, 1]])

	# Define distortion coefficients
	dist_coef = np.array([-0.000233, -0.000328, 0.000116, -0.000326, 0.000000])

	# define image calibration threads
	th_frnt = FrntImgCalib("/auv_v2/f_cam/image_raw",
							args["f_width"], args["f_height"], cam_mtx, dist_coef)
	th_frnt.daemon = True

	th_frnt.start()

	print("Started processing front camera's output...")

	try:
		rospy.spin()
	except:
		# sending stop command
		th_frnt.stop_thread = True
		th_frnt.join()
	finally:
		print("Closing AUV front vision.")