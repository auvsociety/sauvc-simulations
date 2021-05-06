#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox
from sensor_msgs.msg import Image

bridge = CvBridge()
image = None

b_box = BoundingBoxes()
image = []

def point_in_bbox(point, bbox):
	if(point[0] >= bbox.xmin and point[0] <= bbox.xmax 
	and point[1] <= bbox.xmax and point[1] >= bbox.xmin):
		return 1
	else:
		return 0

def verify_gate_coords(green_pole, red_pole, mid_pole, gate):
	red_p = find_bbox_center(red_pole)
	green_p = find_bbox_center(green_pole)
	mid_p = find_bbox_center(mid_pole)
	
def find_bbox_center(bbox):
	return ((bbox.xmin + bbox.xmax)/2, (bbox.ymin + bbox.ymax)/2)

def callback(msg):
	
	green_pole = BoundingBox()
	red_pole = BoundingBox()
	mid_pole = BoundingBox()
	gate = BoundingBox()

	# Take the best prediction for each red, green and mid pole
	for box in msg.bounding_boxes:
		if(box.Class == "red_pole" and box.probability > red_pole.probability):
			red_pole = box
		elif (box.Class == "green_pole" and box.probability > green_pole.probability):
			green_pole = box
		elif (box.Class == "mid_pole" and box.probability > mid_pole.probability):
			mid_pole = box
		elif (box.Class == "gate" and box.probability > gate.probability):
			gate = box

	i = None
	for i in image:
		if msg.header.seq == i.header.seq:
			break

	frame = bridge.imgmsg_to_cv2(i, 'bgr8')

	# print(green_pole, red_pole, mid_pole, gate)
	if(green_pole.Class != '' and red_pole.Class != '' and mid_pole.Class != ''):
		verify_gate_coords(green_pole, red_pole, mid_pole, gate)
		center = find_bbox_center(gate)
		cv2.circle(frame, center, 5, (123, 105, 221), 3)
		print("GATE FOUND")
	else:
		print("GATE NOT FOUND")


	cv2.imshow("Camera Feed", frame)
	cv2.waitKey(1)

	image.remove(i)

def image_callback(msg):
	image.append(msg)

if __name__ == "__main__":

	rospy.init_node('val_gate')
	rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback)
	rospy.Subscriber("/darknet_ros/detection_image", Image, image_callback)

	rospy.spin()