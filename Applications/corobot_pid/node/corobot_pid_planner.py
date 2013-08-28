#!/usr/bin/env python
import roslib; roslib.load_manifest('corobot_pid')
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from corobot_msgs.msg import MotorCommand
import cv2
import cv
import numpy as np
import math
import time
from random import randint
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from math import sin

# Very Very simple planner turns the wheels on when it sees green
# and turns the wheels off when it doesnt

# Hardcoded values for green android folder
LOW_HSV = [60, 50, 50]

HIGH_HSV = [90, 255, 255]

#GLOBAL IMAGES
hsv_img = None
bin_img = None

cvbridge = CvBridge()

target_velocity_pub = None

# Get binary thresholded image
# low_HSV, hi_HSV - low, high range values for threshold as a list [H,S,V]
# debug= True to display the binary image generated
def get_binary(src_img, low_HSV, hi_HSV, debug=False):
    global hsv_img, bin_img
    #convert to HSV
    hsv_img = cv2.cvtColor(src_img, cv.CV_BGR2HSV)
    #generate binary image
    lower = np.array(low_HSV)
    higher = np.array(hi_HSV)
    bin_img = cv2.inRange(hsv_img, lower, higher)
    if debug:
        cv2.namedWindow("Binary")
        cv2.imshow("Binary",bin_img)
        cv2.waitKey(0)
        cv2.destroyWindow("Binary")
        
    return bin_img

def receiveImage(data):
    global target_velocity_pub, cvbridge

    try:
        cv_image = cvbridge.imgmsg_to_cv(data, "bgr8")
    except CvBridgeError, e:
        print e

    arr = np.asarray(cv_image)
    bin_img = get_binary(arr, LOW_HSV, HIGH_HSV)

    imgSize = np.shape(bin_img)

    # NOTE: Assumes an accurate color/marker detection at the very top row of the image
    start = -1
    end = -1
    row = 0

    for j in range(imgSize[1]):
        if start < 0 and bin_img[row,j] != 0:
	        start = j
        if end < 0 and start >= 0 and bin_img[row,j] == 0:
	        end = j
        if (start >= 0 and end >= 0):
	        break

    target_vel = 0
    print start, end
    if start >= 0 or end >= 0:
        target_vel = 50
    else:
        print "No line detected, error_rad = 0"
    target_velocity_pub.publish(target_vel)

def node():
    global target_velocity_pub
    rospy.init_node('cplan')
    rospy.Subscriber('image_raw', Image, receiveImage)
    target_velocity_pub = rospy.Publisher('target_velocity', Int32)
    print "Ready to control the robot"
    rospy.spin()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
