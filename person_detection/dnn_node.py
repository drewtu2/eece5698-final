#!/usr/bin/env python
from __future__ import print_function

import roslib

roslib.load_manifest('person_detection')
import sys
import rospy
import cv2
import imutils
import np
import argparse
import logging

from imutils import paths
from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
from person_detector import person_detector

class dnn_detector(person_detector):
    # Code Modified from
    # https://www.pyimagesearch.com/2017/09/18/real-time-object-detection-with-deep-learning-and-opencv/
    def __init__(self):
        """
        """

        logging.debug("Attempting to create the DNN Detector")

        person_detector.__init__(self)

        # construct the argument parse and parse the arguments
        ap = argparse.ArgumentParser()
        ap.add_argument("-p", "--prototxt", 
                default="real-time-object-detection/MobileNetSSD_deploy.prototxt.txt",
                help="path to Caffe 'deploy' prototxt file")
        ap.add_argument("-m", "--model", 
                default="real-time-object-detection/MobileNetSSD_deploy.caffemodel",
                    help="path to Caffe pre-trained model")
        ap.add_argument("-c", "--confidence", type=float, default=0.4,
                    help="minimum probability to filter weak detections")
        self.args = vars(ap.parse_args())

        # initialize the list of class labels MobileNet SSD was trained to detect
        # then generate a set of bounding box colors for each class
        self.CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat", 
                "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog", 
                "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train",
                "tvmonitor"]
        self.COLORS = np.random.uniform(0, 255, size=(len(self.CLASSES), 3))

        # load our serialized model from disk
        print("[INFO] loading model...")
        self.net = cv2.dnn.readNetFromCaffe(self.args["prototxt"], self.args["model"])
        
        logging.debug("DNN Detector Created...")

    def detect(self, data):
        """
        Detect and classify images inside the given image.

        :args data: a ROS image 
        """
        logging.debug("Attempting to classify items in image")
        
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            logging.warning(e)
        
        frame = imutils.resize(frame, width=400)
        cv2.imshow("Frame", frame)

        logging.debug("Image converted to opencv and resized")

        # grab the frame dimensions and convert it to a blob
        (h, w) = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (224, 224)), 0.007843, (224, 224), 127.5)
                 
        # pass the blob through the network and obtain the detections and
        # predictions
        self.net.setInput(blob)
        detections = self.net.forward()

        logging.debug("Detections and predictions created...")

        rois = []
        
        # loop over the detections
        for i in np.arange(0, detections.shape[2]):
            # extract the confidence (i.e., probability) associated with
            # the prediction
            confidence = detections[0, 0, i, 2]
            
            # filter out weak detections by ensuring the `confidence` is
            # greater than the minimum confidence
            if confidence > self.args["confidence"]:
                # extract the index of the class label from the
                # `detections`, then compute the (x, y)-coordinates of
                # the bounding box for the object
                idx = int(detections[0, 0, i, 1])
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")
                
                # draw the prediction on the frame
                label = "{}: {:.2f}%".format(self.CLASSES[idx], confidence * 100)
                cv2.rectangle(frame, (startX, startY), (endX, endY),
                self.COLORS[idx], 2)
                y = startY - 15 if startY - 15 > 15 else startY + 15
                cv2.putText(frame, label, (startX, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.COLORS[idx], 2)

                if self.CLASSES[idx] is "person":
                    print("found person")
                    rois.append((startX, startY, endX, endY))
        
        logging.debug("attempting to publish... call from dnn_detector")
        super(dnn_detector, self).pub_roi(rois)
        logging.debug("post publish attempt")

        # show the output frame
        cv2.imshow("Frame", frame)
        #key = cv2.waitKey(1) & 0xFF
        cv2.waitKey(3)

        #Only publish if we've detected a person
        #if (len(pick) > 0):
        #    try:
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        #    except CvBridgeError as e:
        #        print(e)

def main(args):
    ic = dnn_detector()
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
    logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.DEBUG)
    global logger
    main(sys.argv)
