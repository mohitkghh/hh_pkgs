#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point

class ImuFFTPSDNode:
    def __init__(self):
        rospy.init_node('imu_fft_psd_publisher')
        
        # Configuration
        self.window_size = rospy.get_param('~window_size', 256)  # Power of 2 recommended
        self.sampling_rate = rospy.get_param('~sampling_rate', 100)  # Hz
        self.publish_rate = rospy.get_param('~publish_rate', 10.0)  # Hz
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
        
        # Publishers
        self.fft_publishers = {
            'x': rospy.Publisher('/imu/fft/x', Float32MultiArray, queue_size=10),
            'y': rospy.Publisher('/imu/fft/y', Float32MultiArray, queue_size=10),
            'z': rospy.Publisher('/imu/fft/z', Float32MultiArray, queue_size=10),
            'freq': rospy.Publisher('/imu/fft/frequencies', Float32MultiArray, queue_size=10, latch=True)
        }
        
        # PSD Publisher
        self.psd_pub = rospy.Publisher('/imu/psd', MarkerArray, queue_size=1)
        
        # Pre-compute and publish frequency bins (latched)
        self.freq_bins = np.fft.fftfreq(self.window_size, 1.0/self.sampling_rate)[:self.window_size//2]
        freq_msg = Float32MultiArray()
        freq_msg.data = self.freq_bins.tolist()
        self.fft_publishers['freq'].publish(freq_msg)
        
        # Subscriber
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        
        # Timer for status checks and PSD publishing
        self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.timer_callback)
        
        rospy.loginfo("IMU FFT+PSD Publisher initialized. Waiting for IMU data...")

    def imu_callback(self, msg):
        current_time = rospy.Time.now()
        
        # Check if this is the first IMU message
        if not self.imu_available:
            self.imu_available = True
            rospy.loginfo("IMU data available. FFT+PSD processing started.")
        
        self.last_imu_time = current_time
        
        # Store data in buffers
        self.buffers['x'][self.buffer_idx] = msg.linear_acceleration.x
        self.buffers['y'][self.buffer_idx] = msg.linear_acceleration.y
        self.buffers['z'][self.buffer_idx] = msg.linear_acceleration.z
        
        self.buffer_idx += 1
        
        # Process when buffer is full
        if self.buffer_idx >= self.window_size:
            self.process_fft()
            self.process_psd()
            self.buffer_idx = 0  # Reset index

    def process_fft(self):
        """Process and publish FFT results"""
        for axis in ['x', 'y', 'z']:
            # Apply window function
            windowed = self.buffers[axis] * np.hamming(self.window_size)
            
            # Compute FFT and get magnitude
            fft_result = np.abs(np.fft.fft(windowed)[:self.window_size//2])
            
            # Publish
            msg = Float32MultiArray()
            msg.data = fft_result.tolist()
            self.fft_publishers[axis].publish(msg)

    def process_psd(self):
        """Calculate and publish PSD from FFT results"""
        if not self.imu_available:
            return
            
        # Create MarkerArray message for PSD
        marker_array = MarkerArray()
        colors = [
            ColorRGBA(1.0, 0.0, 0.0, 1.0),  # Red (X)
            ColorRGBA(0.0, 1.0, 0.0, 1.0),  # Green (Y)
            ColorRGBA(0.0, 0.0, 1.0, 1.0)   # Blue (Z)
        ]
        
        for i, axis in enumerate(['x', 'y', 'z']):
            # Calculate PSD (|FFT|^2 / (N*sampling_rate))
            windowed = self.buffers[axis] * np.hamming(self.window_size)
            fft_result = np.fft.fft(windowed)[:self.window_size//2]
            psd = (np.abs(fft_result)**2) / (self.window_size * self.sampling_rate)
            
            marker = Marker(
                header=Header(
                    frame_id="imu_psd",
                    stamp=self.last_imu_time if self.last_imu_time else rospy.Time.now()
                ),
                ns="psd",
                id=i,
                type=Marker.LINE_STRIP,
                action=Marker.ADD,
                scale=Point(x=0.01, y=0, z=0),
                color=colors[i]
            )
            
            for f, p in zip(self.freq_bins, psd):
                marker.points.append(Point(
                    x=float(f),
                    y=float(np.log10(p)) if p > 0 else -20,
                    z=0
                ))
            
            marker_array.markers.append(marker)
        
        self.psd_pub.publish(marker_array)

    def check_imu_status(self):
        """Check IMU data availability and log warnings if needed"""
        current_time = rospy.Time.now()
        
        if self.last_imu_time is None:
            # Only warn after 2 seconds of initialization
            if (current_time - self.start_time) > rospy.Duration(2.0):
                rospy.logwarn_throttle(5.0, "Waiting for initial IMU data...")
            return False
        
        if (current_time - self.last_imu_time) > self.imu_timeout:
            if self.imu_available:
                rospy.logwarn_throttle(5.0, "IMU data timeout - check IMU connection")
                self.imu_available = False
            return False
        
        if not self.imu_available:
            self.imu_available = True
            rospy.loginfo("IMU data available again. Resuming processing.")
            
        return True

    def timer_callback(self, event):
        """Periodic callback for status checks and maintenance"""
        self.check_imu_status()

if __name__ == '__main__':
    try:
        node = ImuFFTPSDNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down IMU FFT+PSD publisher")