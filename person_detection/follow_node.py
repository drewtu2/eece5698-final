
import rospy
import roslib
from sensor_msgs.msg import RegionOfInterest, CameraInfo 
from geometry_msgs.msg import Twist

class person_follower():
    """
    Based on code from https://github.com/pirobot/ros-by-example/blob/master/rbx_vol_1/rbx1_apps/nodes/object_tracker.py
    """

    def __init__(self):
        """
        Initialize the person following robot
        """

        rospy.init_node("person_follower")

        # Set shutdown function
        rospy.on_shutdown(self.shutdown)

        # How often dow we update the robot's motion?
        self.rate = rospy.get_param("~rate", 10)
        r = rospy.Rate(self.rate)

        # The maximum rotation speed in radians
        # The maximum rotation speed in radians per second
        self.max_rotation_speed = rospy.get_param("~max_rotation_speed", 2.0)
        
        # The minimum rotation speed in radians per second
        self.min_rotation_speed = rospy.get_param("~min_rotation_speed", 0.5)
                        
        # Sensitivity to target displacements.  Setting this too high
        # can lead to oscillations of the robot.
        self.gain = rospy.get_param("~gain", 2.0)
                                                                
        # The x threshold (% of image width) indicates how far off-center
        # the ROI needs to be in the x-direction before we react
        self.x_threshold = rospy.get_param("~x_threshold", 0.1)

        # Publisher to control the robot's movement
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist)
        
        # Intialize the movement command
        self.move_cmd = Twist()
        
        # We will get the image width and height from the camera_info topic
        self.image_width = 320
        self.image_height = 240
        
        # Set flag to indicate when the ROI stops updating
        self.target_visible = False
        
        # Wait for the camera_info topic to become available
        #rospy.loginfo("Waiting for camera_info topic...")
        #rospy.wait_for_message('raspicam_node/camera_info', CameraInfo)

        # Subscribe the camera_info topic to get the image width and height
        #rospy.Subscriber('camera_info', CameraInfo, self.get_camera_info)

        # Wait until we actually have the camera data
        #while self.image_width == 0 or self.image_height == 0:
        #    rospy.sleep(1)

        self.straight_count = 0;
        self.straight_wait = 20;
            
        # Subscribe to the ROI topic and set the callback to update the robot's motion
        rospy.Subscriber('roi', RegionOfInterest, self.set_cmd_vel)
        
        # Wait until we have an ROI to follow
        rospy.wait_for_message('roi', RegionOfInterest)
        
        rospy.loginfo("ROI messages detected. Starting tracker...")
        
        # Begin the tracking loop
        while not rospy.is_shutdown():
            # If the target is not visible, stop the robot
            if not self.target_visible:
                if self.straight_count > self.straight_wait:
                    self.move_cmd = Twist()
                else:
                    self.straight_count += 1
            else:
                # Reset the flag to False by default
                self.target_visible = False
            
            # Send the Twist command to the robot
            self.cmd_vel_pub.publish(self.move_cmd)
            
            # Sleep for 1/self.rate seconds
            r.sleep()

    def set_cmd_vel(self, msg):
        # If the ROI has a width or height of 0, we have lost the target
        if msg.width == 0 or msg.height == 0:
            return
    
        if self.straight_count < self.straight_wait:
            print("Straight and wait")
            self.move_cmd = Twist()
            self.move_cmd.linear.x = 1.5;
            self.straight_count += 1;
            return

        
        # If the ROI stops updating this next statement will not happen
        self.target_visible = True
        
        # Compute the displacement of the ROI from the center of the image
        target_offset_x = msg.x_offset + msg.width / 2 - self.image_width / 2

        try:
            percent_offset_x = float(target_offset_x) / (float(self.image_width) / 2.0)
            print("Target Offset: " + str(target_offset_x))
            print("Percent Offset: " + str(percent_offset_x))
        except:
            percent_offset_x = 0
        
        # Rotate the robot only if the displacement of the target exceeds the threshold
        if abs(percent_offset_x) > self.x_threshold:
            # Set the rotation speed proportional to the displacement of the target
            try:
                speed = self.gain * percent_offset_x
                if speed < 0:
                    direction = 1
                    self.move_cmd.angular.z = -direction * max(self.min_rotation_speed,
                    min(self.max_rotation_speed, abs(speed)))
                else:
                    direction = -1
                    self.move_cmd.angular.z = -direction * max(self.min_rotation_speed,
                    min(self.max_rotation_speed, abs(speed)))
            except:
                self.move_cmd = Twist()
        else:
            # Otherwise stop the robot from rotating and move forward. 
            self.move_cmd = Twist()
            self.move_cmd.linear.x = 1.5;
            self.straight_count = 0;
        
    def get_camera_info(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height
        
        self.image_width = 320
        self.image_height = 240

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)     

if __name__ == '__main__':
    try:
        person_follower()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Quitting")
        rospy.loginfo("Object tracking node terminated.")
