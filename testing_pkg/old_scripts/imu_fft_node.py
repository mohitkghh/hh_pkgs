#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

class ImuFFTNode:
    def __init__(self):
        rospy.init_node('imu_fft_publisher')
        
        # Configuration
        self.window_size = rospy.get_param('~window_size', 256)  # Power of 2 recommended
        self.sampling_rate = rospy.get_param('~sampling_rate', 100)  # Hz
        self.imu_available = False
        self.last_imu_time = None
        self.start_time = rospy.Time.now()
        self.imu_timeout = rospy.Duration(1.0)  # 1 second timeout
        
        # Buffers for each axis
        self.buffers = {
            'x': np.zeros(self.window_size),
            'y': np.zeros(self.window_size),
            'z': np.zeros(self.window_size)
        }
        self.buffer_idx = 0
        
        # Publishers for each axis
        self.publishers = {
            'x': rospy.Publisher('/imu/fft/x', Float32MultiArray, queue_size=10),
            'y': rospy.Publisher('/imu/fft/y', Float32MultiArray, queue_size=10),
            'z': rospy.Publisher('/imu/fft/z', Float32MultiArray, queue_size=10),
            'freq': rospy.Publisher('/imu/fft/frequencies', Float32MultiArray, queue_size=10, latch=True)
        }
        
        # Pre-compute and publish frequency bins (latched)
        self.freq_bins = np.fft.fftfreq(self.window_size, 1.0/self.sampling_rate)[:self.window_size//2]
        freq_msg = Float32MultiArray()
        freq_msg.data = self.freq_bins.tolist()
        self.publishers['freq'].publish(freq_msg)
        
        # Subscriber
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        
        rospy.loginfo("IMU FFT Publisher initialized. Waiting for IMU data...")

    def imu_callback(self, msg):
        current_time = rospy.Time.now()
        
        # Check if this is the first IMU message
        if not self.imu_available:
            self.imu_available = True
            rospy.loginfo("IMU data available. FFT node now running.")
        
        self.last_imu_time = current_time
        
        # Store data in buffers
        self.buffers['x'][self.buffer_idx] = msg.linear_acceleration.x
        self.buffers['y'][self.buffer_idx] = msg.linear_acceleration.y
        self.buffers['z'][self.buffer_idx] = msg.linear_acceleration.z
        
        self.buffer_idx += 1
        
        # Process when buffer is full
        if self.buffer_idx >= self.window_size:
            self.process_fft()
            self.buffer_idx = 0  # Reset index

    def process_fft(self):
        for axis in ['x', 'y', 'z']:
            # Apply window function
            windowed = self.buffers[axis] * np.hamming(self.window_size)
            
            # Compute FFT and get magnitude
            fft_result = np.abs(np.fft.fft(windowed)[:self.window_size//2])
            
            # Publish
            msg = Float32MultiArray()
            msg.data = fft_result.tolist()
            self.publishers[axis].publish(msg)

    def check_imu_status(self):
        current_time = rospy.Time.now()
        
        if self.last_imu_time is None:
            # Only warn after 2 seconds of initialization
            if (current_time - self.start_time) > rospy.Duration(2.0):
                rospy.logwarn_throttle(5.0, "Waiting for IMU data...")
            return False
        
        if (current_time - self.last_imu_time) > self.imu_timeout:
            rospy.logwarn_throttle(5.0, "IMU data timeout - check IMU connection")
            self.imu_available = False
            return False
        
        return True

if __name__ == '__main__':
    try:
        node = ImuFFTNode()
        rate = rospy.Rate(10)  # 10Hz status check
        
        while not rospy.is_shutdown():
            node.check_imu_status()
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass