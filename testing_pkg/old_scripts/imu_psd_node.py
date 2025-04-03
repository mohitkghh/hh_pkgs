#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point

class IMUAccelerationPSDPublisher:
    def __init__(self):
        rospy.init_node('imu_psd_publisher')
        
        # Configurable parameters
        self.window_size = rospy.get_param('~window_size', 1000)  # Samples per PSD calculation
        self.sampling_rate = rospy.get_param('~sampling_rate', 100)  # IMU sample rate (Hz)
        self.publish_rate = rospy.get_param('~publish_rate', 1.0)  # PSD publish rate (Hz)
        
        # Data buffers
        self.accel_x = []
        self.accel_y = []
        self.accel_z = []
        
        # Publisher
        self.psd_pub = rospy.Publisher('/imu/psd', MarkerArray, queue_size=1)
        
        # Subscriber
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        
        # Timer for periodic publishing
        rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.publish_psd)
        
        rospy.loginfo(f"IMU PSD Publisher ready. Window: {self.window_size} samples, Rate: {self.publish_rate} Hz")

    def imu_callback(self, msg):
        """Store IMU acceleration data"""
        self.accel_x.append(msg.linear_acceleration.x)
        self.accel_y.append(msg.linear_acceleration.y)
        self.accel_z.append(msg.linear_acceleration.z)

    def compute_psd(self, signal):
        """Basic PSD calculation using FFT"""
        n = len(signal)
        if n < 2:
            return np.array([]), np.array([])
        
        fft = np.fft.fft(signal)
        psd = np.abs(fft)**2 / (n * self.sampling_rate)
        freq = np.fft.fftfreq(n, 1.0/self.sampling_rate)
        
        # Return only positive frequencies
        return freq[:n//2], psd[:n//2]

    def publish_psd(self, event=None):
        """Publish PSD data as MarkerArray"""
        if len(self.accel_x) < self.window_size:
            return
            
        # Compute PSD for each axis
        freq_x, psd_x = self.compute_psd(self.accel_x[-self.window_size:])
        freq_y, psd_y = self.compute_psd(self.accel_y[-self.window_size:])
        freq_z, psd_z = self.compute_psd(self.accel_z[-self.window_size:])
        
        # Create MarkerArray message
        marker_array = MarkerArray()
        colors = [
            ColorRGBA(1.0, 0.0, 0.0, 1.0),  # Red (X)
            ColorRGBA(0.0, 1.0, 0.0, 1.0),  # Green (Y)
            ColorRGBA(0.0, 0.0, 1.0, 1.0)   # Blue (Z)
        ]
        
        for i, (freq, psd, color) in enumerate(zip(
            [freq_x, freq_y, freq_z],
            [psd_x, psd_y, psd_z],
            colors
        )):
            marker = Marker(
                header=Header(
                    frame_id="imu_psd",
                    stamp=rospy.Time.now()
                ),
                ns="psd",
                id=i,
                type=Marker.LINE_STRIP,
                action=Marker.ADD,
                scale=Point(x=0.01, y=0, z=0),
                color=color
            )
            
            # Add PSD points
            for f, p in zip(freq, psd):
                marker.points.append(Point(
                    x=float(f),
                    y=float(np.log10(p)) if p > 0 else -20,
                    z=0
                ))
            
            marker_array.markers.append(marker)
        
        self.psd_pub.publish(marker_array)
        rospy.logdebug("Published new PSD data")

if __name__ == '__main__':
    try:
        node = IMUAccelerationPSDPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass