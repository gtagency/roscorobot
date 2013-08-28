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

# Hardcoded values for green android folder
LOW_HSV = [60, 50, 50]

HIGH_HSV = [90, 255, 255]

#GLOBAL IMAGES
hsv_img = None
bin_img = None

cvbridge = CvBridge()

# Global variables
error_pub = None
target_velocity_pub = None
motor_pub = None

target_velocity = 0

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
    global error_pub, cvbridge

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

    error_rad = 0
    print start, end
    if start >= 0 or end >= 0:
        # compute the error term by computing the midpoint of
        # the detected line in the image, and calculating
        # the angle from that point (at the top of the image)
        # to the center of the bottom of the image
        det_width = (end - start) / 2
        tgt_width = imgSize[1] / 2
        error_px  = (start + det_width) - tgt_width
        error_rad = math.tan(float(error_px)/imgSize[0])

        #print start, end
        #print det_width, tgt_width
        #print error_px, error_rad
    else:
        print "No line detected, error_rad = 0"
    error_pub.publish(error_rad)

def node():
    global error_pub,target_velocity_pub,motor_pub
    rospy.init_node('cpid')
    rospy.Subscriber('image_raw', Image, receiveImage)
    error_pub = rospy.Publisher('control_error', Float64)
    rospy.Subscriber('target_velocity', Int32, set_target_velocity)
    motor_pub = rospy.Publisher('PhidgetMotor', MotorCommand)
    rospy.Subscriber('control_correction', Float64, send_command)
    print "Ready to accept images and send errors"
    rospy.spin()

def set_target_velocity(target):
    global target_velocity
    target_velocity = target.data
    print "New target velocity", target_velocity

def send_command(adjustment):
    global target_velocity, motor_pub
    # Only move if the target velocity is non zero
    # Otherwise, the robot should be stationary
    if target_velocity != 0:
        mc = MotorCommand()
        ang_f = adjustment.data
        tv_f = float(target_velocity)
        mc.leftSpeed  = tv_f + (tv_f/2) * sin(ang_f)
        mc.rightSpeed = tv_f + (tv_f/2) * (-sin(ang_f))
        mc.acceleration = 50
        mc.secondsDuration = 1
        print "Publishing corrected motor command", mc
    #    motor_pub.publish(mc)

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
