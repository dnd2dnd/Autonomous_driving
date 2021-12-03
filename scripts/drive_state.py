#!/usr/bin/env python

import rospy, time
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from blocking_bar import Blocking_Bar
from robotController import RobotController
from smach import State, StateMachine
from lane_follower import LaneTrace
from slane_follower import SLaneTrace
from DetectStopSign import DetectStopSign
from detect_box import Detectbox
from follower import Follower
# from detect_stop_line import DetectStopLine
import smach_ros
class ReadyToStart(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.change_time = time.time()

    def execute(self, userdata):
        robot_controller = RobotController()
        while True:
            robot_controller.set_velocity(0.9)
            robot_controller.set_angular(0)
            robot_controller.drive()
            if self.change_time + 3< time.time():
                robot_controller.set_velocity(0)
                robot_controller.set_angular(0)
                robot_controller.drive()
                return 'success'
is_blocking_bar=False
def bar_callback(msg):
    global is_blocking_bar
    if msg.data=='GO':
        is_blocking_bar = True
    if msg.data=='NO':
        is_blocking_bar = False
bar_sub = rospy.Subscriber('detect/is_block', String, bar_callback)
class DetectBlockingBar(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        global is_blocking_bar
        self.change_time = time.time()
        is_blocking_bar=True
    def execute(self, ud):
        rate = rospy.Rate(20)
        robot_controller=RobotController()
        while True:
            if is_blocking_bar==True:
                change_time=time.time()
                # while change_time + 3 > time.time():
                #     robot_controller.set_velocity(0.9)
                #     robot_controller.set_angular(0)
                #     robot_controller.drive()
                return 'success'
                # if self.change_time + 2 < time.time():
                #     rospy.loginfo("z")
                #     return 'success'
            rate.sleep()

global lane_tracer

is_stop_line=False
def stop_callback(msg):
    global is_stop_line
    if msg.data=='STOP':
        is_stop_line = True
    if msg.data=='NO':
        is_stop_line = False
stop_sub = rospy.Subscriber('stop_line', String, stop_callback)

stop_line_count=0
class Follower(State):
    def __init__(self):
        global is_stop_line
        global lane_tracer
        State.__init__(self, outcomes=['success','going', 'going2'])
        self.is_success=False
        self.is_go= False
        lane_tracer=LaneTrace()
        self.area_sub = rospy.Subscriber('area', String, self.area_callback)
        self.area=''
    def area_callback(self,msg):
        self.area=msg.data
    def execute(self, ud):
        global stop_line_count
        rate=rospy.Rate(20)
        robot_controller = RobotController()

        while True:
            print('area:',self.area,stop_line_count)
            if stop_line_count==4:
                return 'going'
                # lane_tracer.start_line_trace('COURSE',0.05)
            if stop_line_count>=6:
                return 'going2'
            if self.is_success:
                return 'success'
            if is_stop_line==True:
                change_time=time.time()
                stop_line_count=stop_line_count+1
                # while change_time +1 > time.time():
                rospy.sleep(3)
            lane_tracer.start_line_trace('COURSE')
            rate.sleep()

class Straightgo(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.change_time=time.time()
        self.test=False

    def execute(self, ud):
        toggle=False
        global stop_line_count
        rate=rospy.Rate(20)
        robot_controller = RobotController()
        while True:
            change_time = time.time()
            if stop_line_count>=4:
                print('go',change_time,time.time())
                while change_time + 9 >time.time():
                    robot_controller.set_velocity(0.8)
                    robot_controller.set_angular(0.1)
                    robot_controller.drive()
                    print('gogogogo')
                toggle=True
            if toggle==True:
                stop_line_count=0
                return 'success'

global slane_tracer
class SFollower(State):
    def __init__(self):
        global is_stop_line
        global slane_tracer
        global lane_tracer
        lane_tracer=LaneTrace()
        State.__init__(self, outcomes=['success'])
        self.is_success=False
        self.is_go= False
        slane_tracer = SLaneTrace()
        self.area_sub = rospy.Subscriber('area', String, self.area_callback)
        self.area=''
        self.count=0
    def area_callback(self,msg):
        self.area=msg.data
    def execute(self, ud):
        global stop_line_count
        rate=rospy.Rate(20)
        robot_controller = RobotController()
        while True:
            print('area:',self.area,'stop_count',stop_line_count)
            # if stop_line_count>=4:
            #     return 'going'
                # lane_tracer.start_line_trace('COURSE',0.05)
            if is_stop_line==True:
                change_time=time.time()
                stop_line_count = stop_line_count + 1
                if self.count==1:
                    rospy.sleep(3)
                    change_time=time.time()
                    while change_time + 10 > time.time():
                        robot_controller.set_velocity(0.8)
                        robot_controller.set_angular(0.05)
                        robot_controller.drive()
                    stop_line_count=5
                    return 'success'
                if stop_line_count==4:
                    while change_time + 3 > time.time():
                        robot_controller.set_velocity(0.5)
                        robot_controller.set_angular(-0.3)
                        robot_controller.drive()
                if stop_line_count==5:
                    while change_time + 6 > time.time():
                        robot_controller.set_velocity(0.8)
                        robot_controller.set_angular(-0.2)
                        robot_controller.drive()
                        self.count=1
                else:
                    while change_time +1 > time.time():
                        rospy.sleep(1)
            if self.count==0:
                slane_tracer.start_line_trace('COURSE')
            else:
                lane_tracer.start_line_trace('COURSE')
            rate.sleep()
stop_sign_toggle=None
global detect_stop
global detect_box
global box_time
class YFollower(State):
    def __init__(self):
        global is_blocking_bar
        global stop_line_count
        global is_stop_line
        global lane_tracer
        global detect_box
        global slane_tracer
        State.__init__(self, outcomes=['success'])
        self.is_success=False
        self.is_go= False
        global detect_stop
        detect_stop=DetectStopSign()
        detect_box=Detectbox()
        lane_tracer = LaneTrace()
        slane_tracer = SLaneTrace()
        self.stop_sign_sub = rospy.Subscriber('stop_sign', String, self.area_callback)
        self.contours=0
    def area_callback(self,msg):
        global stop_sign_toggle
        if msg.data == 'STOP_SIGN':
            stop_sign_toggle = True
        if msg.data == 'NO_STOP_SIGN':
            stop_sign_toggle = False

    def execute(self, ud):
        stop_line_count=0
        rate=rospy.Rate(20)
        count=0
        robot_controller = RobotController()
        boxstop=0
        while True:
            print(self.contours)
            check = 0
            if stop_sign_toggle==True:
                count+=1
                print('stop')
                rospy.sleep(3)
                change_time=time.time()
                while change_time + 3 > time.time():
                    robot_controller.set_velocity(0.8)
                    robot_controller.set_angular(0.05)
                    robot_controller.drive()
            if detect_box.range_ahead >=0:
                print('box stop')
                rospy.sleep(1)
                boxstop+=1
                box_time=time.time()
            if boxstop>=4 and box_time+30<time.time():
                print('next')
                robot_controller.set_velocity(0.9)
                robot_controller.drive()
                return 'success'
            else:
                robot_controller.set_velocity(0.9)
                robot_controller.drive()

            # if stop_line_count>=1 and len(detect_stop.contours) >15:
            #     while change_time + 6 > time.time():
            #         print('qwerasd')
            #         robot_controller.set_velocity(0.8)
            #         robot_controller.set_angular(-0.3)
            #         robot_controller.drive()


                # while change_time + 10 > time.time():
                #     print('qwert')
                #     robot_controller.set_velocity(0.6)
                #     robot_controller.drive()
            if self.contours>=3 and stop_line_count>=1:
                robot_controller.set_velocity(0.2)
                robot_controller.set_angular(-0.8)
                robot_controller.drive()
                check=2
            if check==0:
                lane_tracer.start_line_trace('COURSE')
            elif check==1:
                print('qq')
                slane_tracer.start_line_trace('LeftCourse')
            # slane_tracer.start_line_trace('LeftCourse')
            rate.sleep()

class TFollower(State):

    def __init__(self):
        global is_blocking_bar
        global stop_line_count
        global is_stop_line
        global lane_tracer
        global detect_box
        global slane_tracer
        State.__init__(self, outcomes=['success'])
        self.is_success=False
        self.is_go= False
        global detect_stop
        detect_stop=DetectStopSign()
        detect_box=Detectbox()
        lane_tracer = LaneTrace()
        slane_tracer = SLaneTrace()
        self.contours_pub = rospy.Subscriber('contours', String, self.area_callback)
        self.contours=0
        self.stop_sign_sub = rospy.Subscriber('stop_sign', String, self.area_callback2)
    def area_callback2(self,msg):
        global stop_sign_toggle
        if msg.data == 'STOP_SIGN':
            stop_sign_toggle = True
        if msg.data == 'NO_STOP_SIGN':
            stop_sign_toggle = False
    def area_callback(self,msg):
        self.contours=float(msg.data)
    def execute(self, ud):
        stop_line_count=0
        rate=rospy.Rate(20)
        count=0
        robot_controller = RobotController()
        check = 0
        while True:

            print('contours',self.contours)
            if stop_sign_toggle==True:
                print('stop')
                rospy.sleep(3)
                change_time=time.time()
                return 'success'
            # if self.contours>= 3000 and self.contours <= 4000:
            #     rospy.sleep(3)
            if is_stop_line==True:
                stop_line_count+=1
                rospy.sleep(3)
                change_time=time.time()
                # while change_time + 2 > time.time():
                #     robot_controller.set_velocity(0.9)
                #     robot_controller.set_angular(0)
                #     robot_controller.drive()
                while change_time + 8 > time.time():
                    robot_controller.set_velocity(0.6)
                    robot_controller.set_angular(-0.3)
                    robot_controller.drive()
                while change_time + 16 > time.time():
                    print('e')
                    robot_controller.set_velocity(0.6)
                    robot_controller.set_angular(-0.3)
                    robot_controller.drive()
                check=1
            if self.contours>=3 and stop_line_count>=1:
                change_time=time.time()
                while change_time + 2 > time.time():
                    robot_controller.set_velocity(0.2)
                    robot_controller.set_angular(-0.8)
                    robot_controller.drive()
                check=0
            if check==0:
                lane_tracer.start_line_trace('COURSE')
            elif check==1:
                slane_tracer.start_line_trace('LeftCourse')
            # slane_tracer.start_line_trace('LeftCourse')
            rate.sleep()