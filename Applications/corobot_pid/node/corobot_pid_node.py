#!/usr/bin/env python
import roslib; roslib.load_manifest('corobot_pid')
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from corobot_msgs.msg import MotorCommand
from math import sin

# Global variables
target_velocity = None
motor_pub = None


def node():
    global target_velocity, motor_pub
    rospy.init_node('cpid')
    error_pub = rospy.Publisher('control_error', Float32)
    rospy.Subcriber('target_velocity', Int32, set_target_velocity)
    motor_pub = rospy.Publisher('PhidgetMotor', MotorCommand)
    rospy.Subcriber('control_correction', Float32, send_command)

    rospy.spin()


def set_target_velocity(target):
    global target_velocity
    target_velocity = target


def send_command(adjustment):
    global target_velocity, motor_pub
    mc = MotorCommand()
    mc.leftSpeed = target_velocity * sin(adjustment)
    mc.rightSpeed = target_velocity * (1 - sin(adjustment))
    mc.acceleration = 50
    mc.secondsDuration = 1
    motorPub.publish(mc)

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
