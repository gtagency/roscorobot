#!/usr/bin/env python

import rospy
import dead_reckoning_localizer
from geometry_msgs.msg import Pose2D
from corobot_msgs.msg import MotorCommand

class DeadReckoningNode:
    def __init__(wheelRadius, wheelSeparation, speedAdjustment):
        deadReckoningLocalizer = dead_reckoning_localizer.DeadReckoningLocalizer(wheelRadius, wheelSeparation, speedAdjustment)
        self.leftSpeed = 0
        self.rightSpeed = 0
        self.lastUpdateTime = rospy.get_time()
        self.posPub = rospy.Publisher("car_position", Pose2D)
        rospy.Subscriber("PhidgetMotor", MotorCommand, self.updateSpeed)

    def run(self):
        r = rospy.Rate(25)
        while not rospy.is_shutdown():
            self.updatePosition();
            r.sleep()

    def updateSpeed(self, command):
        self.leftSpeed = command.leftSpeed
        self.rightSpeed = command.rightSpeed

    def updatePosition(self):
        currTime = rospy.get_time()
        msg = Pose2D()
        msg.x, msg.y, msg.theta = deadReckoningLocalizer.update(self.leftSpeed, self.rightSpeed, currTime - self.lastUpdateTime)
        self.posPub.publish(msg)
        self.lastUpdateTime = currTime


if __name__ == "__main__":
    wheelRadius = rospy.get_param('wheelRadius', 0)
    wheelSeparation = rospy.get_param('wheelSeparation', 0)
    speedAdjustment = rospy.get_param('speedAdjustment', 0)

    deadReckoning = DeadReckoningNode(wheelRadius, wheelSeparation, speedAdjustment)
    deadReckoning.run()