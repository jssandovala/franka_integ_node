#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from franka_msgs.msg import FrankaState
from cv_bridge import CvBridge
import cv2
import time
import os

class IntegVisionNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('integ_vision_node', anonymous=True)

        # Subscribers
        # TO DO

        # Init variables
        self.bridge = CvBridge()
        self.latest_image = None
        self.current_cartesian_pose = None

        # Define the folder to save images
        self.save_folder = "/home/franka/Pictures/integ_saved_images"
        if not os.path.exists(self.save_folder):
            os.makedirs(self.save_folder)

        # Define the file to save Cartesian poses
        self.cart_poses_file = os.path.join(self.save_folder, "cart_poses.txt")
        with open(self.cart_poses_file, 'w') as f:
            f.write("Image Name, TCP poses (t00, t10, t20, t30, t10, t11, t12, t13, t20, t21, t22, t23, t30, t31, t32, t33) \n")

    def actions_callback(self, msg):
        # Activate fonctions (e.g. launch self.save_image_and_robot_state() to record a single image+pose)
        print('TO DO')
    
    def cart_states_callback(self, msg):
        # Store the current Cartesian pose of the robot
        print('TO DO')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image and store it
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"Failed to process image: {e}")

    def save_image_and_robot_state(self):
        if self.latest_image is None:
            rospy.logwarn("No image received yet. Camera might not be active.")
            return

        if self.current_cartesian_pose is None:
            rospy.logwarn("No Cartesian pose received yet. Robot might not be active.")
            return

        try:
            # Save the image & Save the Cartesian pose to the file
            print("TO DO")

        except Exception as e:
            rospy.logerr(f"Failed to save image or robot state: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = IntegVisionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
