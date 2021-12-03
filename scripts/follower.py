#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class Follower:
    def __init__(self,image_topic_name,dir):
        self.bridge = cv_bridge.CvBridge()
        # cv2.namedWindow("window"+image_topic_name, 1)
        self.image_sub = rospy.Subscriber(image_topic_name, Image, self.image_callback)
        self.area_pub = rospy.Publisher('area', String, queue_size=1)
        # self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
        #                                Twist, queue_size=1)
        # self.twist = Twist()
        self.cx = 0
        self.err = 0
        self.image_topic_name = image_topic_name
        self.contours=[]
        self.dir=dir
        self.area=0
    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_white = numpy.array([0, 0, 200])
        upper_white = numpy.array([180, 255, 255])

        mask=cv2.inRange(hsv,lower_white,upper_white)
        h, w, d = image.shape
        search_top = 1 * h / 2
        mask[0:search_top, 0:w] = 0
        if self.dir=='left':
            mask[0:h, 60:w] = 0
        else:
            mask[0:h, 0:560]=0
        M = cv2.moments(mask)
        ret, thr = cv2.threshold(mask,127,255,0)
        _, self.contours, _ = cv2.findContours(thr, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # self.area_pub.publish(str(area))
        if M['m00'] > 0:
            self.area = max(list(map(lambda x: cv2.contourArea(x), self.contours)))
            self.cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(image, (self.cx, cy), 20, (0, 0, 255), -1)
            self.cx = self.cx
        # if self.dir=='right':
        #     cv2.imshow("window"+self.image_topic_name, image)
        #     cv2.imshow("mask"+self.image_topic_name, mask)
        #     cv2.waitKey(3)
        # if self.dir=='left':
        #     cv2.imshow("window"+self.image_topic_name, image)
        #     cv2.imshow("mask"+self.image_topic_name, mask)
        #     cv2.waitKey(3)