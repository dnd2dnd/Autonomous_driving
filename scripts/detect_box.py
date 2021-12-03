#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from robotController import RobotController
class Detectbox:
    def __init__(self):
        self.range_ahead = 0
        self.range_right = 0
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.drive_controller = RobotController()

    def scan_callback(self,msg):
        self.range_ahead = msg.ranges[len(msg.ranges) / 2]
        self.range_right = max(msg.ranges[160],msg.ranges[240])

def scan_callback(msg):
    range_ahead = msg.ranges[len(msg.ranges)/2]
    print("range ahead :", range_ahead)
