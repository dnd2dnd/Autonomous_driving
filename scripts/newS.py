#! /usr/bin/env python

import rospy, time
from std_msgs.msg import String, Float32

from robotController import RobotController
from smach import State, StateMachine
from slane_follower import SLaneTrace

class newS(State):
    def __init__(self):
        global is_stop_line
        global slane_tracer
        State.__init__(self, outcomes=['success'])
        self.is_success = False
        self.is_go = False

        self.area_sub = rospy.Subscriber('area', String, self.area_callback)
        self.area = ''


    def area_callback(self, msg):
        self.area = msg.data


    def execute(self, ud):
        global stop_line_count
        slane_tracer = SLaneTrace()
        rate = rospy.Rate(20)
        robot_controller = RobotController()

        while True:
            print('z')
            # print('area:',self.area,stop_line_count)
            # if stop_line_count>=4:
            #     return 'going'
            # lane_tracer.start_line_trace('COURSE',0.05)
            if self.is_success:
                return 'success'
            # if is_stop_line == True:
            #     change_time = time.time()
            #     stop_line_count = stop_line_count + 1
                while change_time + 3 > time.time():
                    rospy.sleep(1)
            print('q')
            slane_tracer.start_line_trace(True)
            print('q')
            rate.sleep()