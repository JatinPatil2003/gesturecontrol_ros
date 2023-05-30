#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def display_video(data):
    try:
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)

    cv2.imshow('Video', frame)
    cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('video_subscriber', anonymous=True)

    rospy.Subscriber('camera/image_raw', Image, display_video)

    # cv2.namedWindow('Video', cv2.WINDOW_NORMAL)

    while not rospy.is_shutdown():
        rospy.spin()

    cv2.destroyAllWindows()
