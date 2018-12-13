#!/usr/bin/env python
from __future__ import print_function

import roslib

roslib.load_manifest('person_detection')
import sys
import rospy
import cv2
import imutils
import np
import logging

from imutils import paths
from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class person_detector(object):

    def __init__(self):
        logging.debug("attempting to create person detector base calss.")
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("raspicam2/image_raw", Image, self.detect)
        self.image_pub = rospy.Publisher("detected", Image)
        self.image_pub_roi = rospy.Publisher("roi", RegionOfInterest)
        
        logging.debug("created person detector base class")

    def detect(self, image):
        logging.debug("Override detect in base class")
        
    def pub_roi(self, l_roi):
        """
        Publish the roi of the maximum sized area
        :args l_roi: a list of RegionOfInterst Objects
        """
        
        logging.debug("attempting to publish to the roi topic")
        #print("attempting to publish to the roi topic")

        # Short circuit of no rois
        if len(l_roi) == 0: 
            logging.debug("no roi to publish")
            print("no roi to publish")
            return

        try:
            largest = max(l_roi, key=lambda p : p[2] * p[3]);
            
            # Short Circuit if invalid dimensions
            if largest[0] < 0 or largest[1] < 0:
                return

            roi = RegionOfInterest()
            roi.x_offset = largest[0] 
            roi.y_offset = largest[1] 
            roi.width = largest[2] 
            roi.height = largest[3] 
            
        except Exception as e:
            logging.warning(e)
            print()
            print(e)
            print(largest)
            print()
            roi = RegionOfInterest()

        try:
            self.image_pub_roi.publish(roi)
            logging.debug("ROI published")
            print("ROI published")
            roi = RegionOfInterest()
        except Exception as e:
            logging.warning(e)
            print()
            print(e)
            print(largest)
            print()
            roi = RegionOfInterest()
            return









