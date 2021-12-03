#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from robotController import RobotController

class SFollower:
    def __init__(self,image_topic_name,dir):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber(image_topic_name, Image, self.image_callback)
        self.cx = 0
        self.err = 0
        self.image_topic_name = image_topic_name
        self.contours=0
        self.dir=dir
    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_white = numpy.array([0, 0, 200])
        upper_white = numpy.array([180, 255, 255])

        mask=cv2.inRange(hsv,lower_white,upper_white)
        h, w, d = image.shape
        search_top = 1 * h / 2
        mask[0:search_top, 0:w] = 0
        mask[0:h, 0:560] = 0

        # if self.dir=='left':
        #     mask[0:h, 60:w] = 0
        # else:
        #     mask[0:h, 0:560]=0
        M = cv2.moments(mask)
        ret, thr = cv2.threshold(mask,127,255,0)
        _, self.contours, _ = cv2.findContours(thr, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # print(self.contours)
        # print(len(self.contours))
        if len(self.contours)==0:
            print('@', self.contours)

            self.drive_controller.set_velocity(0.2)
            self.drive_controller.set_angular(-0.8)
            self.drive_controller.drive()
            count = 1
        if M['m00'] > 0:
            self.cx = int(M['m10'] / M['m00']) - 10
            cy = int(M['m01'] / M['m00'])
            cv2.circle(image, (self.cx, cy), 20, (0, 0, 255), -1)
            self.cx = self.cx
            if len(self.contours) >= 1:
                cx = self.cx - w/2
                err = float(cx) / 1200
                print(self.cx, cx, err,self.area)
                if count == 0:
                    if self.area >= 6000:
                        print('k')
                        self.drive_controller.set_velocity(0.1)
                        self.drive_controller.set_angular(0.7)
                        self.drive_controller.drive()
                    if abs(err) >= 1:
                        self.drive_controller.set_velocity(0.1)
                        self.drive_controller.set_angular(err)
                        self.drive_controller.drive()
                    elif abs(err) >= 0.5:
                        self.drive_controller.set_velocity(0.2)
                        self.drive_controller.set_angular(err)
                        self.drive_controller.drive()
                    elif abs(err) >= 0.3:
                        self.drive_controller.set_velocity(0.2)
                        self.drive_controller.set_angular(err)
                        self.drive_controller.drive()
                    elif abs(err) < 0.3:
                        self.drive_controller.set_velocity(0.2)
                        self.drive_controller.set_angular(err)
                        self.drive_controller.drive()
            else:
                print(self.contours)
                cx = self.cx - 200
                err = float(cx) / 1000
                self.drive_controller.set_velocity(0.2)
                self.drive_controller.set_angular(-0.8)
                self.drive_controller.drive()
        if self.dir=='right':
            cv2.imshow("window"+self.image_topic_name, image)
            cv2.imshow("mask"+self.image_topic_name, mask)
            cv2.waitKey(3)
        # if self.dir=='left':
        #     cv2.imshow("window"+self.image_topic_name, image)
        #     cv2.imshow("mask"+self.image_topic_name, mask)
        #     cv2.waitKey(3)

if __name__ =='__main__':
    rospy.init_node('test')
    drive=Follower('right_camera/rgb/image_raw','right')
    while not rospy.is_shutdown():
        rospy.spin()