# Turtlebot Followbot, EECE 5698 Final Project
This is the final project for EECE 5698, Robotics Sensing and Navigation with 
Prof. Singh. The goal of this project was to develop a robot capable of following
a user using various methods of person detection on the Turtlebot3 Platform. 

## Presentation: 
https://drive.google.com/open?id=1gFGnLI1dI_8AC7MUfZJt8i2YP8WJrGdVNFDrBbliSAA

## Person Detection
We evaluated methods of person detection using both LIDAR and camera sensors.

### Camera
We implemented two forms of person detection using camera. Each method pulled 
ROS Image messages from the `/raspicam2/image_raw` topic and published the ROI 
to follow to the `/roi` topic. If more than one ROI was found, only the ROI with 
the largest area was published. 

**HOG Detector**: The first method we used was OpenCVs built in pedestrian HoG
Detector. After detection, we perform a non-maxima supression to prevent the same
object from being detected more than once. While this method worked, it proved 
fairly inconsistent, and required the pedestrian be in ideal lighting and focus 
conditions. Any occlusion or change in position severely impacted the
performance of the detector. Furthermore, the HoG Detector also had a fair number
of false positives.

![Hog Detector][HOG]

**MobilenetSSD**: The second detector we used was the MobileNetSSD network using
[OpenCV 3.3s built in DNN support][1]. This network proved to be much more accurate
and was capable of detecting people in a number of different positions, including
when parts of the body were occluded. Furthermore, MobileNet was able to classify
a number of other object, in addition to people. While we did not use these other
labels, this feature could be an interesting topic to explore in future projects. 

![MobileNet][MobileNet]

### Lidar
We used a pre-trained random forest classifier to detect a person using the sensor
data from the Turtlebot's LIDAR. While this classifier was originally trained to
detect a person at 50cm, we were able to modify some of the filter parameters 
to detect a person up to 1m. Unfortunately, the LIDAR on the turtlebot is an
inexpensive, low resolution 2D LIDAR that has a range of 3.5m and only takes
one point/degree. With such a low resolution detecting a person beyond 1m becomes
exceptionally difficult. 

## Setup Dependencies
- [turtlebot3][https://github.com/ROBOTIS-GIT/turtlebot3]
- [turtlebot3\_msgs][https://github.com/ROBOTIS-GIT/turtlebot3_msgs]
- [raspicam\_node][https://github.com/UbiquityRobotics/raspicam_node]
- [gscam (for use with GStreamer)][https://github.com/ros-drivers/gscam]
- OpenCV 3.3

### Remote PC Side
- Need to be running the NTP server to allow the TurtleBot to synchronize its 
timing when offline
- Ensure `$ROS_MASTER_URI` is set to the ip address of your machine
- Run `roscore` on the remote pc side. The remote machine is the master node.
- Launch `turtlebot3_remote` node
- Launch `gscam` node
- Run whichever detector node you want (dnn or hog)
- Run follow_bot node (reads /roi, publishes to /cmd_vel)

### TurtleBot Side
- Ensure `$ROS_MASTER_URI` is set to the ip address of your remote machine
- Ensure the date and time are correctly set (i.e. synchronized) with the master
node
- Run turtlebot3_bringup node
- Stream the video from the Raspberry picam to the GStreamer Node

[1]: https://github.com/opencv/opencv/wiki/Deep-Learning-in-OpenCV
[HOG]: presentation/hog.png
[MobileNet]: presentation/dnn.png