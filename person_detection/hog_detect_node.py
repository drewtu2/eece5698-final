#!/usr/bin/env python
from __future__ import print_function

import roslib

roslib.load_manifest('person_detection')
import sys
import rospy
import cv2
import imutils
import np

from imutils import paths
from imutils.object_detection import non_max_suppression
from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class person_detector:

    def __init__(self):

        self.bridge = CvBridge()
        #self.image_sub = rospy.Subscriber("raspicam_node/image/image_raw", Image,self.callback)
        self.image_sub = rospy.Subscriber("raspicam2/image_raw", Image, self.detect)
        self.image_pub = rospy.Publisher("detected", Image)
        self.image_pub_roi = rospy.Publisher("roi", RegionOfInterest)

        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def detect(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        # load the image and resize it to (1) reduce detection time
        # and (2) improve detection accuracy
        frame = imutils.resize(frame, width=min(400, frame.shape[1]))
        orig = frame.copy()

        # detect people in the image
        (rects, weights) = self.hog.detectMultiScale(frame, winStride=(4, 4),
        padding=(8, 8), scale=1.05)

        # draw the original bounding boxes
        for (x, y, w, h) in rects:
            cv2.rectangle(orig, (x, y), (x + w, y + h), (0, 0, 255), 2)

        # apply non-maxima suppression to the bounding boxes using a
        # fairly large overlap threshold to try to maintain overlapping
        # boxes that are still people
        rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
        pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)

        # Create a list of the ROIs
        rois = []

        # draw the final bounding boxes
        for (xA, yA, xB, yB) in pick:
            cv2.rectangle(frame, (xA, yA), (xB, yB), (0, 255, 0), 2)
            center = ((xA + xB)/2, (yA + yB)/2)
            rois.append((xA, yA, xB, yB))
         
        # publish the rois
        self.pub_roi(rois)

        # show some information on the number of bounding boxes
        print("[INFO] {}: {} original boxes, {} after suppression".format(
        "feed", len(rects), len(pick)))

        # show the output images
        cv2.imshow("Before NMS", orig)
        cv2.imshow("After NMS", frame)

        cv2.waitKey(3)

        # Only publish if we've detected a person
        if (len(pick) > 0):
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            except CvBridgeError as e:
                print(e)
    
        
    def pub_roi(self, l_roi):
        """
        Publish the roi
        """
        # Short circuit of no rois
        if len(l_roi) == 0:
            return

        largest = max(l_roi, key=lambda p : p[2] * p[3]);
        print("largest: " + str(largest))
        roi = RegionOfInterest()
        roi.x_offset = largest[0] 
        roi.y_offset = largest[1] 
        roi.width = largest[2] 
        roi.height = largest[3] 
        print(roi)
        self.image_pub_roi.publish(roi)
        roi = RegionOfInterest()


def main(args):
    ic = person_detector()
    rospy.init_node('image_converter', anonymous=True)
    print("running")
    try:
        print("stuff is happening!")
        rospy.spin()
        print("stuff is not happening!")
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
