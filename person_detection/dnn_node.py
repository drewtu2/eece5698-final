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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class person_detector:
    # https://www.pyimagesearch.com/2017/09/18/real-time-object-detection-with-deep-learning-and-opencv/
    def __init__(self):

        self.bridge = CvBridge()
        #self.image_sub = rospy.Subscriber("raspicam_node/image/image_raw", Image,self.callback)
        self.image_sub = rospy.Subscriber("raspicam2/image_raw", Image,self.callback)
        self.image_pub = rospy.Publisher("detected", Image)

        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        # construct the argument parse and parse the arguments
        ap = argparse.ArgumentParser()
        ap.add_argument("-p", "--prototxt", required=True,
                    help="path to Caffe 'deploy' prototxt file")
        ap.add_argument("-m", "--model", required=True,
                    help="path to Caffe pre-trained model")
        ap.add_argument("-c", "--confidence", type=float, default=0.2,
                    help="minimum probability to filter weak detections")
        args = vars(ap.parse_args())

        # initialize the list of class labels MobileNet SSD was trained to detect
        # then generate a set of bounding box colors for each class
        self.CLASSES = ["background", "bicycle", "bird", "boat", "bottle", "bus"
                "car", "cat", "chair", "cow", "diningtable", "dog", "horse", 
                "motorbike", "person", "pottedplant", "sheep", "sofa", "train",
                "tvmonitor"]
        self.COLORS = np.random.uniform(0, 255, size=(len(self.CLASSES), 3))

        # load our serialized model from disk
        print("[INFO] loading model...")
        net = cv2.dnn.readNetFromCaffe(args["prototxt"], args["model"])
        

    def callback(self,data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        # grab the frame dimensions and convert it to a blob
        (h, w) = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)),
        0.007843, (300, 300), 127.5)
                 
        # pass the blob through the network and obtain the detections and
        # predictions
        self.net.setInput(blob)
        detections = self.net.forward()

        # loop over the detections
        for i in np.arange(0, detections.shape[2]):
            # extract the confidence (i.e., probability) associated with
            # the prediction
            confidence = detections[0, 0, i, 2]
            # filter out weak detections by ensuring the `confidence` is
            # greater than the minimum confidence
            if confidence > args["confidence"]:
                # extract the index of the class label from the
                # `detections`, then compute the (x, y)-coordinates of
                # the bounding box for the object
                idx = int(detections[0, 0, i, 1])
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")
                
                # draw the prediction on the frame
                label = "{}: {:.2f}%".format(CLASSES[idx],
                confidence * 100)
                cv2.rectangle(frame, (startX, startY), (endX, endY),
                COLORS[idx], 2)
                y = startY - 15 if startY - 15 > 15 else startY + 15
                cv2.putText(frame, label, (startX, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)

        # Only publish if we've detected a person
        if (len(pick) > 0):
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            except CvBridgeError as e:
                print(e)

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
