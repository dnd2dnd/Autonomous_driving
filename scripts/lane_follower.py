#! /usr/bin/env python
import rospy
from follower import Follower
from robotController import RobotController
class LaneTrace:
    def __init__(self):
        self.left_line = Follower('left_camera/rgb/image_raw','left')
        self.right_line = Follower('right_camera/rgb/image_raw','right')
        self.drive_controller = RobotController()
        self.right_line.contourse=[]
    def right_area(self):
        return self.right_line.contours
    def start_line_trace(self, flag, angle=0):
        count=0
        if angle == 0:
            if flag=='COURSE':
                if len(self.left_line.contours) >= 1 and len(self.right_line.contours) <= 0:
                    self.drive_controller.set_velocity(0.2)
                    self.drive_controller.set_angular(-0.7)
                    self.drive_controller.drive()
                    count = 1
                if len(self.left_line.contours) <= 0 and len(self.right_line.contours) >= 1:
                    self.drive_controller.set_velocity(0.2)
                    self.drive_controller.set_angular(0.7)
                    self.drive_controller.drive()
                    count = 1
                if len(self.left_line.contours) <= 0 and len(self.right_line.contours) <= 0:
                    cx = (self.left_line.cx + self.right_line.cx) / 2 - 320
                    err = -float(cx) / 10
                    self.drive_controller.set_velocity(0.2)
                    self.drive_controller.set_angular(err)
                    self.drive_controller.drive()
                    count = 1
                if len(self.left_line.contours) >= 1 and len(self.right_line.contours) >= 1:
                    cx = (self.left_line.cx + self.right_line.cx) / 2 - 320
                    err = -float(cx) / 80
                    print(cx, err)
                    if count == 0:
                        if abs(err) >= 1:
                            self.drive_controller.set_velocity(0.1)
                            self.drive_controller.set_angular(err*3)
                            self.drive_controller.drive()
                        elif abs(err) >= 0.5:
                            self.drive_controller.set_velocity(0.2)
                            self.drive_controller.set_angular(err*2)
                            self.drive_controller.drive()
                        elif abs(err) >= 0.3:
                            self.drive_controller.set_velocity(0.4)
                            self.drive_controller.set_angular(err)
                            self.drive_controller.drive()
                        elif abs(err) < 0.3:
                            self.drive_controller.set_velocity(0.7)
                            self.drive_controller.set_angular(err)
                            self.drive_controller.drive()
        else:
            self.drive_controller.set_velocity(0.7)
            angle+=angle
            self.drive_controller.set_angular(angle)
            self.drive_controller.drive()
