#! /usr/bin/env python
import cv2, numpy, rospy, cv_bridge, time
from sensor_msgs.msg import Image
from std_msgs.msg import String
class Stop_Line():
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.stop_line_toggle = None
        self.stop_line_pub = rospy.Publisher('stop_line', String, queue_size=1)
        self.area_pub = rospy.Publisher('area', String, queue_size=1)
        self.contours_pub = rospy.Publisher('contours', String, queue_size=1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.area = 0
    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_white = numpy.array([0, 0, 200])
        upper_white = numpy.array([180, 255, 255])
        # v = cv2.split(hsv)[2]  # GRAY
        # mask = cv2.inRange(v, 217, 250)

        mask=cv2.inRange(hsv,lower_white,upper_white)
        h, w, d = image.shape

        # h, w = mask.shape
        mask[0: 380, 0: w] = 0
        mask[0:h, 0:140] = 0
        mask[0:h, 500 :  w] = 0
        ret, thr = cv2.threshold(mask, 127, 255, 0)

        _,contours,_ = cv2.findContours(thr, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        rospy.loginfo('qqq')
        print('kkk')
        if len(contours) > 0:
            cnt = contours[len(contours) - 1]
            self.area = max(list(map(lambda x: cv2.contourArea(x), contours)))
            if self.area >= 9000:
                rospy.loginfo('stop')
                self.stop_line_pub.publish('STOP')
                self.area_pub.publish(str(self.area))
                self.contours_pub.publish(str(len(contours)))
            else:
                self.stop_line_pub.publish('NO')
                self.area_pub.publish(str(self.area))
                self.contours_pub.publish(str(len(contours)))
            print('contourse',len(contours))
            M = cv2.moments(mask)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
        cv2.imshow("window", image)
        cv2.imshow('mask', mask)
        cv2.waitKey(3)

if __name__ == "__main__":
    rospy.init_node('stop_line')
    stop_line = Stop_Line()
    rospy.spin()