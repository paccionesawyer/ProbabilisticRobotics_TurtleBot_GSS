#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import time 

BRIDGE = CvBridge()

sub_topic = "/camera/depth/image_raw"

sub_msg_type = Image


class DepthSub:
    '''
    Simple code for subscribing in ROS
    '''

    def __init__(self, namespace=""):
        '''
        Purpose
        -------
        initialized by calling the Node constructor, naming our node 
        'boiler_plate'
        '''
        rospy.init_node('depth_sub')
        self.subscription   = rospy.Subscriber(sub_topic, sub_msg_type, self.listener_callback)
        
        # cv2.namedWindow("Raw Depth Image")
        # cv2.namedWindow("Array Depth Image")
        # cv2.namedWindow("Normalized Depth Image", cv2.cv.CV_WINDOW_NORMAL)

    def listener_callback(self, data):
        '''
        Purpose
        -------
        Whenever our subscriber (listener) get's a message this function is 
        'called back' to and ran.
        '''
        img_msg = data
        try:
            cv_image = BRIDGE.imgmsg_to_cv2(img_msg, desired_encoding="32FC1")
        except CvBridgeError as e:
            print(e)

        

        cv_image_array = np.array(cv_image, dtype = np.dtype('f8'))
        cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
        width, height = np.shape(cv_image_norm)
        # cv2.imshow("Raw Depth Image", cv_image)
        # cv2.imshow("Array Depth Image", cv_image_array)
        # cv2.imshow("Depth Image", cv_image_norm)
        print(cv_image_norm[width//2 - 5: width//2 + 5, height//2 - 5:height//2 + 5])
        time.sleep(1)
        # cv2.waitKey(1)
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", cv_image_norm)

def main(args=None):
    depth_sub = DepthSub()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
    finally:
        print("Done")


if __name__ == '__main__':
    main()