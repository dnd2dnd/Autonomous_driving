#! /usr/bin/env python
import rospy
from follower import Follower
from robotController import RobotController
class SLaneTrace:
    def __init__(self):
        self.right_line = Follower('right_camera/rgb/image_raw','right')
        self.left_line = Follower('left_camera/rgb/image_raw', 'left')
        self.drive_controller = RobotController()
    def start_line_trace(self, flag, angle=0):
        count=0
        if angle == 0:
            if flag=='COURSE' :
                if len(self.right_line.contours) <= 0:
                    self.drive_controller.set_velocity(0.2)
                    self.drive_controller.set_angular(-0.8)
                    self.drive_controller.drive()
                    count = 1
                if len(self.right_line.contours) >= 1:
                    cx = self.right_line.cx - 120
                    err = float(cx) / 8000
                    print(self.right_line.area, cx, err)
                    if self.right_line.area >=2000:
                        # print('qwe')
                        self.drive_controller.set_velocity(0.1)
                        self.drive_controller.set_angular(0.9)
                        self.drive_controller.drive()
                        count=1
                    if count == 0:
                        # if self.area >= 6000:
                        #     print('k')
                        #     self.drive_controller.set_velocity(0.1)
                        #     self.drive_controller.set_angular(0.7)
                        #     self.drive_controller.drive()
                        self.drive_controller.set_velocity(0.2)
                        self.drive_controller.set_angular(err)
                        self.drive_controller.drive()
                # else:
                #     print('qweqwe')
                #     self.drive_controller.set_velocity(0.2)
                #     self.drive_controller.set_angular(-0.9)
                #     self.drive_controller.drive()
            elif flag=='COURS':
                if len(self.right_line.contours) <= 0:
                    self.drive_controller.set_velocity(0.2)
                    self.drive_controller.set_angular(-0.8)
                    self.drive_controller.drive()

                elif len(self.right_line.contours) >= 1:
                    cx = self.right_line.cx - 20
                    err = float(cx) / 10000
                    print(self.right_line.area, cx, err)
                    if self.right_line.area >=2000:
                        self.drive_controller.set_velocity(0.1)
                        self.drive_controller.set_angular(0.9)
                        self.drive_controller.drive()
                        count=1
                    if count == 0:
                        self.drive_controller.set_velocity(0.4)
                        self.drive_controller.set_angular(err)
                        self.drive_controller.drive()
            elif flag=='LeftCourse':
                print('area',self.left_line.area)
                if len(self.left_line.contours) <= 0:
                    self.drive_controller.set_velocity(0.2)
                    self.drive_controller.set_angular(0.7)
                    self.drive_controller.drive()

                elif len(self.left_line.contours) >= 1:
                    cx = self.left_line.cx + 20
                    err = float(cx) / 400
                    print(self.left_line.area, cx, err)
                    self.drive_controller.set_velocity(0.3)
                    self.drive_controller.set_angular(-err)
                    self.drive_controller.drive()