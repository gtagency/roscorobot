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

cvbridge = CvBridge()

# Global variables
error_pub = None
target_velocity_pub = None
motor_pub = None
edge_pub = None
lines_pub = None
target_velocity = 0


def receiveImage(data):
    global error_pub, cvbridge, edge_pub

    try:
        cv_image = cvbridge.imgmsg_to_cv(data, "bgr8")
    except CvBridgeError, e:
        print e

    arr = np.asarray(cv_image)
    gray_img = cv2.cvtColor(arr, cv2.COLOR_BGR2GRAY)
    thresh, thresh_img = cv2.threshold(gray_img, 128, 255, cv2.THRESH_BINARY|cv2.THRESH_OTSU)
    edge_img = cv2.Canny(thresh_img, 80, 120)
    cv_image2 = cv.fromarray(edge_img)
    edge_pub.publish(cvbridge.cv_to_imgmsg(cv_image2, "passthrough"))
 
    lines = cv2.HoughLines(edge_img, 1, math.pi/180, 100)
    if lines is not None:
        thetas = [theta if (theta < np.pi/2) else (np.pi-theta) for rho, theta in lines[0]]

        avg_theta = sum(thetas)/float(len(thetas))
        error_rad = avg_theta
    else:
        print "No lines detected"
        error_rad = 0
    error_pub.publish(error_rad)

def node():
    global error_pub,target_velocity_pub,motor_pub,edge_pub
    rospy.init_node('cpid')
    rospy.Subscriber('image_raw', Image, receiveImage)
    error_pub = rospy.Publisher('control_error', Float64)
    rospy.Subscriber('target_velocity', Int32, set_target_velocity)
    motor_pub = rospy.Publisher('PhidgetMotor', MotorCommand)
    rospy.Subscriber('control_correction', Float64, send_command)
    edge_pub = rospy.Publisher('edge_img', Image)
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
        motor_pub.publish(mc)

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
