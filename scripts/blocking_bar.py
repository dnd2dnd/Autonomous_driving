#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy

from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point
from robotController import RobotController

class Blocking_Bar:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                          Image, self.image_callback)
        self.detect_block_pub = rospy.Publisher('detect/is_block', String, queue_size=1)

        self.len_contours=0
    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red = numpy.array([-10, 100, 100])
        upper_red = numpy.array([10, 255, 255])

        mask = cv2.inRange(hsv, lower_red, upper_red)
        h, w, d = image.shape

        mask[0:80, 0:w] = 0
        ret, thr = cv2.threshold(mask,127,255,0)
        _, contours, _ = cv2.findContours(thr, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        self.len_contours=len(contours)
        if len(contours)<=2:
            self.detect_block_pub.publish('GO')
        else:
            self.detect_block_pub.publish('NO')
        # cv2.imshow("window", image)
        # cv2.imshow("mask", mask)
        # cv2.waitKey(3)
if __name__ == '__main__':
    rospy.init_node('blocking_bar')
    detecter = Blocking_Bar()
    rospy.spin()
    # while not rospy.is_shutdown():
    #     rospy.spin()


