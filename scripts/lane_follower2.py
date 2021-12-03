#!/usr/bin/env
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image

class LineTracer2:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.left_image_sub = rospy.Subscriber('left_camera/rgb/image_raw', Image, self.left_image_callback)
        self.right_image_sub = rospy.Subscriber('right_camera/rgb/image_raw', Image, self.right_image_callback)
        self.cx_white = 0
        self.cy_white=0
        self.cx_yellow=0
        self.cy_yellow=0

    def left_image_callback(self,msg):

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_white = numpy.array([0, 40, 135])#10 40 135
        upper_white = numpy.array([135, 135, 255])#135 135 255
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        h, w, d = image.shape
        search_top = 3 * h /8
        search_bot = search_top + 20

        mask_white[0:search_top, 0:w] = 0
        mask_white[search_bot:h, 0:w] = 0

        M_w = cv2.moments(mask_white)

        if M_w['m00'] > 0:
            self.cx_white = int(M_w['m10'] / M_w['m00'])
            self.cy_white = int(M_w['m01'] / M_w['m00'])

        # cv2.circle(image, (self.cx_white, self.cy_white), 20, (0, 255, 255), -1)
        # cv2.imshow('mask', image)
        # cv2.waitKey(3)

    def right_image_callback(self,msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_yellow = numpy.array([10, 40, 135])
        upper_yellow = numpy.array([135, 135, 255])

        mask_yellow = cv2.inRange(hsv,lower_yellow ,upper_yellow)

        h, w, d = image.shape
        search_top = 3 * h / 8
        search_bot = search_top + 20

        mask_yellow[0:search_top, 0:w] = 0
        mask_yellow[search_bot:h, 0:w] = 0


        M_y = cv2.moments(mask_yellow)

        if M_y['m00'] > 0:
            self.cx_yellow = int(M_y['m10'] / M_y['m00'])
            self.cy_yellow = int(M_y['m01'] / M_y['m00'])

        # cv2.circle(image, (self.cx_yellow, self.cy_yellow), 20, (0, 255, 255), -1)
        # cv2.imshow('mask', image)
        # cv2.waitKey(3)
