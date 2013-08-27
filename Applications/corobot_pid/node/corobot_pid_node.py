#!/usr/bin/env python
import roslib; roslib.load_manifest('corobot_pid')
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from corobot_msgs.msg import MotorCommand

def node():
    rospy.init_node('cpid')
    pub = rospy.Publisher('control_error', Float32)
    pub = rospy.Publisher('target_velocity', Int32)
    pub = rospy.Publisher('PhidgetMotor', MotorCommand)
    while not rospy.is_shutdown():
	mc = MotorCommand()
	mc.leftSpeed = 50
	mc.rightSpeed = 50
	mc.acceleration = 50
	mc.secondsDuration = 1
        pub.publish(mc)
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



