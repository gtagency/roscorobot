#!/usr/bin/env python
import roslib; roslib.load_manifest('corobot_pid')
import rospy
from std_msgs.msg import Float64

pub = None

def handle_control_error(data):
    global pub
    pub.publish(data.data)

def node():
    global pub
    rospy.init_node('pid_loopback')
    s = rospy.Subscriber('control_error', Float64, handle_control_error)
    pub = rospy.Publisher('control_correction', Float64)
    print "Ready to loopback error -> correction."
    rospy.spin()

if __name__ == '__main__':
    node()
