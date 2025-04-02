#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Vector3Stamped, TwistStamped, PoseStamped, TwistWithCovarianceStamped
# from waterlinked_a50_ros_driver.msg import DVL  # Assuming the DVL message is available in Python

class VelocityCorrectionNode:
    def __init__(self):
        rospy.init_node('velocity_correction_node')

        # Initialize variables
        self.angular_velocity = np.zeros(3)
        self.angular_velocity_time = None
        self.offset = np.array([0.54688, 0.0, 0.09214])  # (del_x, del_y, del_z) offset of AHRS from  in global frame
        self.c1 = np.array([1.0, 1.0, 1.0])       # scaling vector for Raw DVL velocity
        self.imu_data_received = False
        self.dvl_data_received = False

        # Initialize pose (x, y, z) and time
        self.pose = np.zeros(3)  # Initial pose [x, y, z]
        self.last_time = rospy.Time.now()  # Timestamp when the node starts

        # Subscribers
        self.angular_velocity_sub = rospy.Subscriber(
            "imu/angular_velocity", Vector3Stamped, self.angular_velocity_callback
        )
        self.dvl_velocity_sub = rospy.Subscriber(
            "/nfbv/twist/velocity", TwistWithCovarianceStamped, self.dvl_velocity_callback
        )

        # Publishers
        self.corrected_velocity_pub = rospy.Publisher(
            "plant/corrected_velocity", TwistStamped, queue_size=10
        )
        self.pose_pub = rospy.Publisher(
            "plant/pose_from_corrected_vel", PoseStamped, queue_size=10
        )

        # Timer to check data availability
        self.check_data_timer = rospy.Timer(rospy.Duration(1.0), self.check_data_availability)

    def angular_velocity_callback(self, msg):
        """Callback for IMU angular velocity data."""
        self.angular_velocity = np.array([msg.vector.x, msg.vector.y, msg.vector.z])
        self.angular_velocity_time = msg.header.stamp
        self.imu_data_received = True

    def dvl_velocity_callback(self, msg):
        """Callback for DVL velocity data."""
        if not self.imu_data_received:
            rospy.logwarn_throttle(5, "Angular velocity data not yet received. Waiting for IMU data.")
            return

        # Extract Raw DVL velocity from TwistWithCovarianceStamped
        dvl_velocity = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])

        # Scale raw DVL velocity based on a scaling vector
        scaled_dvl_velocity = np.multiply(self.c1, dvl_velocity)

        # Correct velocity using the formula: corrected_velocity = dvl_velocity - (angular_velocity x offset)
        corrected_velocity = scaled_dvl_velocity - np.cross(self.angular_velocity, self.offset)

        # Publish corrected velocity as TwistStamped
        velocity_twist_msg = TwistStamped()
        velocity_twist_msg.header.stamp = rospy.Time.now()
        velocity_twist_msg.header.frame_id = "dvl_link"
        velocity_twist_msg.twist.linear.x = corrected_velocity[0]
        velocity_twist_msg.twist.linear.y = corrected_velocity[1]
        velocity_twist_msg.twist.linear.z = corrected_velocity[2]
        self.corrected_velocity_pub.publish(velocity_twist_msg)

        # Integrate velocity to compute pose
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()  # Time difference in seconds
        self.last_time = current_time

        # Update pose: pose = pose + velocity * dt
        self.pose += corrected_velocity * dt

        # Publish pose as PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = current_time
        pose_msg.header.frame_id = "dvl_link"
        pose_msg.pose.position.x = self.pose[0]
        pose_msg.pose.position.y = self.pose[1]
        pose_msg.pose.position.z = self.pose[2]
        self.pose_pub.publish(pose_msg)

        self.dvl_data_received = True
        rospy.loginfo_throttle(5, f"Successfully published corrected velocity and pose data.")

    def check_data_availability(self, event):
        """Timer callback to check if data is being received."""
        if not self.imu_data_received:
            rospy.logwarn("No data received on topic 'imu/angular_velocity'. Check IMU connection.")
        if not self.dvl_data_received:
            rospy.logwarn("No data received on topic '/nfbv/twist/velocity'. Check DVL connection.")
        if self.imu_data_received and self.dvl_data_received:
            rospy.loginfo_throttle(10, "Receiving data from both IMU and DVL. Node is functioning normally.")

if __name__ == '__main__':
    try:
        node = VelocityCorrectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
