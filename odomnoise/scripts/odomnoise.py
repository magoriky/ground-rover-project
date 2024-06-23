#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, PoseStamped, TwistStamped
from numpy.random import normal
import numpy as np

class OdometryNoiser:
    def __init__(self):
        rospy.init_node('odometry_noiser')

        # Subscribe to the odometry topic
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Publish the noisy odometry to a different topic
        self.noisy_odom_pub = rospy.Publisher('/noisyodom', Odometry, queue_size=10)

    def odom_callback(self, msg):
        # Add noise to the odometry data
        noisy_msg = self.add_noise_to_odom(msg)

        # Publish the noisy odometry
        self.noisy_odom_pub.publish(noisy_msg)

    def add_noise_to_odom(self, msg):
        # Clone the original odometry message
        noisy_msg = msg

        # Add noise to the position
        noisy_msg.pose.pose.position.x += normal(0, 0.3)
        noisy_msg.pose.pose.position.y += normal(0, 0.3)
        noisy_msg.pose.pose.position.z += normal(0, 0.3)

        # Add noise to the orientation (quaternion)
        # Note: This is a simple example; you may want to add more sophisticated noise modeling
        noisy_msg.pose.pose.orientation.x += normal(0, 0.3)
        noisy_msg.pose.pose.orientation.y += normal(0, 0.3)
        noisy_msg.pose.pose.orientation.z += normal(0, 0.3)
        noisy_msg.pose.pose.orientation.w += normal(0, 0.3)

        # Add noise to the linear velocity
        noisy_msg.twist.twist.linear.x += normal(0, 0.3)
        noisy_msg.twist.twist.linear.y += normal(0, 0.3)
        noisy_msg.twist.twist.linear.z += normal(0, 0.3)

        # Add noise to the angular velocity
        noisy_msg.twist.twist.angular.x += normal(0, 0.3)
        noisy_msg.twist.twist.angular.y += normal(0, 0.3)
        noisy_msg.twist.twist.angular.z += normal(0, 0.3)

        # Add noise to the covariance matrix
        noisy_msg.pose.covariance = np.diag([1e-3,1e-3,1e-3,1e-3,1e-3,3e2])   
        noisy_msg.twist.covariance = np.diag([1e-3,1e-3,1e-3,1e-3,1e-3,3e2])   

        return noisy_msg



if __name__ == '__main__':
    try:
        odometry_noiser = OdometryNoiser()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
