# import the necessary packages
from imutils.object_detection import non_max_suppression
from imutils import paths
import numpy as np
import argparse
import imutils
import cv2
 
# construct the argument parse and parse the arguments
#ap = argparse.ArgumentParser()
#ap.add_argument("-i", "--images", required=True, help="path to images directory")
#args = vars(ap.parse_args())
 
# Get a reference to webcame #0 (default webcam)
video_capture = cv2.VideoCapture(0)

# initialize the HOG descriptor/person detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
while True:
    ret, frame = video_capture.read()

    # load the image and resize it to (1) reduce detection time
    # and (2) improve detection accuracy
    frame = imutils.resize(frame, width=min(400, frame.shape[1]))
    orig = frame.copy()
                     
    # detect people in the image
    (rects, weights) = hog.detectMultiScale(frame, winStride=(4, 4),
    padding=(8, 8), scale=1.05)
    
    # draw the original bounding boxes
    for (x, y, w, h) in rects:
        cv2.rectangle(orig, (x, y), (x + w, y + h), (0, 0, 255), 2)
                                                 
    # apply non-maxima suppression to the bounding boxes using a
    # fairly large overlap threshold to try to maintain overlapping
    # boxes that are still people
    rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
    pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)
                                                                     
    # draw the final bounding boxes
    for (xA, yA, xB, yB) in pick:
        cv2.rectangle(frame, (xA, yA), (xB, yB), (0, 255, 0), 2)
        center = ((xA + xB)/2, (yA + yB)/2)
    
    # show some information on the number of bounding boxes
    print("[INFO] {}: {} original boxes, {} after suppression".format(
    "feed", len(rects), len(pick)))
    
    # show the output images
    cv2.imshow("Before NMS", orig)
    cv2.imshow("After NMS", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
video_capture.release()
cv2.destroyAllWindows()
