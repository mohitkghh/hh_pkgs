#!/usr/bin/env python3

import rospy
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import TwistWithCovarianceStamped, TwistStamped, Twist, Vector3
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class OdometryFusion:
    def __init__(self):
        rospy.init_node('odometry_fusion_node', anonymous=True)
        
        # Initialize variables
        self.position = np.zeros(3)  # x, y, z in odom frame
        self.velocity = np.zeros(3)  # body-frame velocity
        self.orientation = np.zeros(4)  # quaternion
        self.angular_velocity = np.zeros(3)
        self.linear_acceleration = np.zeros(3)
        self.last_time = rospy.Time.now()
        
        # DVL data
        self.dvl_velocity = np.zeros(3)
        self.dvl_available = False
        
        # IMU data
        self.imu_available = False
        
        # Publishers
        self.odom_pub = rospy.Publisher('/plant/odometry', Odometry, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Subscribers
        rospy.Subscriber('/plant/corrected_velocity', TwistStamped, self.dvl_callback)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        
        # Parameters
        
        self.global_ned_frame = rospy.get_param('~global_ned_frame', 'global_ned')
        self.base_link_frame = rospy.get_param('~base_link_frame', 'base_link')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.publish_tf = rospy.get_param('~publish_tf', True)
        
        rospy.loginfo("Odometry fusion node initialized")
    
    def dvl_callback(self, msg):
        """Callback for DVL velocity data"""
        self.dvl_velocity = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z
        ])
        self.dvl_available = True
        self.update_odometry()
    
    def imu_callback(self, msg):
        """Callback for IMU data"""
        # Orientation
        self.orientation = np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])
        
        # Angular velocity
        self.angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        # Linear acceleration
        self.linear_acceleration = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        
        self.imu_available = True
        self.update_odometry()
    
    def update_odometry(self):
        """Fuse DVL and IMU data to update odometry estimate"""
        if not (self.dvl_available and self.imu_available):
            return
            
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time
        
        if dt <= 0:
            return
            
        try:
            # # Get rotation matrix from quaternion
            # (roll, pitch, yaw) = euler_from_quaternion(self.orientation)
            # rot_matrix = tf.transformations.euler_matrix(roll, pitch, yaw)[:3, :3]
            
            # # Transform DVL velocity from body frame to world frame
            # world_velocity = np.dot(rot_matrix, self.dvl_velocity)

            world_velocity = self.dvl_velocity  # the dvl velocity is already in world frame
            
            # Simple integration for position update
            self.position += world_velocity * dt
            
            # Create and publish odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time
            odom_msg.header.frame_id = self.odom_frame
            odom_msg.child_frame_id = self.base_link_frame
            
            # Set position
            odom_msg.pose.pose.position.x = self.position[0]
            odom_msg.pose.pose.position.y = self.position[1]
            odom_msg.pose.pose.position.z = self.position[2]
            
            # Set orientation
            odom_msg.pose.pose.orientation.x = self.orientation[0]
            odom_msg.pose.pose.orientation.y = self.orientation[1]
            odom_msg.pose.pose.orientation.z = self.orientation[2]
            odom_msg.pose.pose.orientation.w = self.orientation[3]
            
            # Set velocity (in child frame)
            odom_msg.twist.twist.linear.x = self.dvl_velocity[0]
            odom_msg.twist.twist.linear.y = self.dvl_velocity[1]
            odom_msg.twist.twist.linear.z = self.dvl_velocity[2]
            odom_msg.twist.twist.angular.x = self.angular_velocity[0]
            odom_msg.twist.twist.angular.y = self.angular_velocity[1]
            odom_msg.twist.twist.angular.z = self.angular_velocity[2]
            
            # Publish odometry message
            self.odom_pub.publish(odom_msg)
            
            # Publish transform if enabled
            if self.publish_tf:
                # Create and publish the static global_NED to odom transform
                # This converts odom (XYZ) to NED convention by rotating 180° around X
                static_ned_transform = TransformStamped()
                static_ned_transform.header.stamp = rospy.Time.now()
                static_ned_transform.header.frame_id = self.global_ned_frame  # "global_NED"
                static_ned_transform.child_frame_id = self.odom_frame         # "odom"
                
                # Position is zero (frames are coincident)
                static_ned_transform.transform.translation.x = 0.0
                static_ned_transform.transform.translation.y = 0.0
                static_ned_transform.transform.translation.z = 0.0
                
                # Rotation: 180° around X axis (π radians)
                # This converts:
                # X (Forward) -> X (North)
                # Y (Left)    -> -Y (East)
                # Z (Up)      -> -Z (Down)
                ned_rotation = tf.transformations.quaternion_from_euler(3.141593, 0, 0)  # PI about X
                
                static_ned_transform.transform.rotation.x = ned_rotation[0]
                static_ned_transform.transform.rotation.y = ned_rotation[1]
                static_ned_transform.transform.rotation.z = ned_rotation[2]
                static_ned_transform.transform.rotation.w = ned_rotation[3]
                
                # Publish the static transform
                self.tf_broadcaster.sendTransform(static_ned_transform)
                
                # Create and publish the dynamic odom to base_link transform
                dynamic_transform = TransformStamped()
                dynamic_transform.header.stamp = current_time
                dynamic_transform.header.frame_id = self.odom_frame
                dynamic_transform.child_frame_id = self.base_link_frame
                
                dynamic_transform.transform.translation.x = self.position[0]
                dynamic_transform.transform.translation.y = self.position[1]
                dynamic_transform.transform.translation.z = self.position[2]
                
                dynamic_transform.transform.rotation.x = self.orientation[0]
                dynamic_transform.transform.rotation.y = self.orientation[1]
                dynamic_transform.transform.rotation.z = self.orientation[2]
                dynamic_transform.transform.rotation.w = self.orientation[3]
                
                self.tf_broadcaster.sendTransform(dynamic_transform)

                
        except Exception as e:
            rospy.logerr("Error in odometry update: %s", str(e))
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = OdometryFusion()
        node.run()
    except rospy.ROSInterruptException:
        pass