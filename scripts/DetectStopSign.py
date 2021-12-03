#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy

from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point
from robotController import RobotController

class DetectStopSign:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.stop_sign_pub = rospy.Publisher('stop_sign', String, queue_size=1)
        self.stop_sign_check = None
        self.contours=[]

    def image_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red = numpy.array([0, 0, 90])
        upper_red = numpy.array([5, 5, 110])

        mask = cv2.inRange(hsv, lower_red, upper_red)
        h, w, d = image.shape

        # mask[h:620, 0:w] = 0
        # mask[0:h, 0:460] = 0
        # mask[0:h/2,0:w]=0

        ret, thr = cv2.threshold(mask, 127, 255, 0)
        _, self.contours, _ = cv2.findContours(thr, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        # cv2.imshow("window", image)
        # cv2.imshow("mask", mask)
        # cv2.waitKey(3)

        if len(self.contours)>=1:
            if cv2.contourArea(self.contours[len(self.contours) - 1]) >= 180:
                # print('abc', cv2.contourArea(self.contours[len(self.contours) - 1]))
                self.stop_sign_check = 'STOP_SIGN'
                self.stop_sign_pub.publish(self.stop_sign_check)
            else:
                # print('abc', cv2.contourArea(self.contours[len(self.contours) - 1]))
                self.stop_sign_check = 'NO_STOP_SIGN'
                self.stop_sign_pub.publish(self.stop_sign_check)

        # block_bar_mask[0:h, 0: w- w/5] = 0
        # block_bar_mask[h/2:h, 0:w] = 0
        # block_bar_mask[0:h/10, 0:w] = 0
        # ret, thr = cv2.threshold(mask, 127, 255, 0)
        # _, self.contours, _ = cv2.findContours(thr, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        # cv2.imshow("window"+self.image_topic_name, image)
        # cv2.imshow("mask", origin_image)
        # cv2.imshow("hsv", hsv)
        # cv2.waitKey(3)