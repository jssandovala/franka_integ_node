#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import TwistStamped
from franka_msgs.msg import FrankaState
from keyboard.msg import Key
from cv_bridge import CvBridge
import cv2

class IntegVisualServoingNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('integ_visual_servoing_node', anonymous=True)

        # Subscribers
        rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        rospy.Subscriber('/franka_state_controller/joint_states', JointState, self.joint_states_callback)
        rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.cart_states_callback)

        # Publisher
        self.cart_vel_pub = rospy.Publisher('/cart_del_desired_', TwistStamped, queue_size=10)

        # Rate of the loop 
        self.rate = rospy.Rate(10)

        self.bridge = CvBridge()
        self.latest_image = None
        self.current_joint_positions = None
        self.current_cartesian_pose = None

    def joint_states_callback(self, msg):
        self.current_joint_positions = msg.position  # Tuple of joint positions

    def cart_states_callback(self, msg):
        # Store the current Cartesian pose of the robot
        pose = msg.O_T_EE
        self.current_cartesian_pose = (
            pose[0], pose[1], pose[2], pose[3], 
            pose[4], pose[5], pose[6], pose[7], 
            pose[8], pose[9], pose[10], pose[11], 
            pose[12], pose[13], pose[14], pose[15]
        )

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image and store it
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"Failed to process image: {e}")

    def run(self):
        while not rospy.is_shutdown():
            
            ##############################
            # Develop your custom code here
            ##############################
            
            #Example of publishing zero velocity
            twist_msg = TwistStamped()
            twist_msg.header.stamp = rospy.Time.now()
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.linear.y = 0.0
            twist_msg.twist.linear.z = 0.0
            twist_msg.twist.angular.x = 0.0
            twist_msg.twist.angular.y = 0.0
            twist_msg.twist.angular.z = 0.0
            self.cart_vel_pub.publish(twist_msg)
            rospy.loginfo("Published zero Cartesian velocity")

        self.rate.sleep()

if __name__ == '__main__':
    try:
        node = IntegVisualServoingNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
