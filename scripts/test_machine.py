#! /usr/bin/env python

import smach_ros
import rospy
from smach import StateMachine
from drive_state import ReadyToStart, DetectBlockingBar, Follower, Straightgo, SFollower, YFollower, TFollower
from newS import newS
class AutoDriving:
    def __init__(self):
        self.test_machine = StateMachine(outcomes=['success'])
    def auto_drive(self):
        rospy.init_node('test')
        with self.test_machine:
            StateMachine.add('Bar',ReadyToStart(), transitions={'success':'DetectBlockingBar'})
            StateMachine.add('DetectBlockingBar', DetectBlockingBar(), transitions={'success': 'Follower'})
            StateMachine.add('Follower', Follower(), transitions={'going': 'go','success': 'success','going2':'Follower3'})
            StateMachine.add('go', Straightgo(), transitions={'success': 'Follower2'})
            StateMachine.add('Follower2', SFollower(), transitions={'success': 'Follower3'})
            StateMachine.add('Follower3', YFollower(), transitions={'success': 'Follower4'})
            StateMachine.add('Follower4', TFollower(), transitions={'success': 'success'})
        self.test_machine.execute()
        rospy.spin()
if __name__ =='__main__':
    drive=AutoDriving()
    drive.auto_drive()