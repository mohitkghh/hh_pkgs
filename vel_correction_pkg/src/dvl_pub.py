#!/usr/bin/env python3
import socket
import json
import rospy
from time import sleep
from std_msgs.msg import String, Float64

import select
import math

from geometry_msgs.msg import TwistWithCovarianceStamped


def connect():
	global s, TCP_IP, TCP_PORT
	try:
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		s.connect((TCP_IP, TCP_PORT))
		s.settimeout(1)
	except socket.error as err:
		rospy.logerr("No route to host, DVL might be booting? {}".format(err))
		sleep(1)
		connect()

oldJson = ""

altitude = Float64()

velocity_twist_msg = TwistWithCovarianceStamped()
def getData():
	global oldJson, s
	raw_data = ""

	while not '\n' in raw_data:
		try:
			rec = s.recv(1) # Add timeout for that
			if len(rec) == 0:
				rospy.logerr("Socket closed by the DVL, reopening")
				connect()
				continue
		except socket.timeout as err:
			rospy.logerr("Lost connection with the DVL, reinitiating the connection: {}".format(err))
			connect()
			continue
		rec = rec.decode()	
		raw_data = raw_data + rec
	raw_data = oldJson + raw_data
	oldJson = ""
	raw_data = raw_data.split('\n')
	oldJson = raw_data[1]
	raw_data = raw_data[0]
	return raw_data


def publisher():
	velocity_pub = rospy.Publisher('nfbv/velocity',Float64,queue_size=10)
	altitude_pub = rospy.Publisher('nfbv/altitude',Float64,queue_size=10)
	velocity_twist_pub = rospy.Publisher('nfbv/twist/velocity',TwistWithCovarianceStamped,queue_size=10)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		raw_data = getData()
		data = json.loads(raw_data)

		# edit: the logic in the original version can't actually publish the raw data
		# we slightly change the if else statement so now
		# do_log_raw_data is true: publish the raw data to /dvl/json_data topic, fill in theDVL using velocity data and publish to dvl/data topic
		# do_log_raw_data is true: only fill in theDVL using velocity data and publish to dvl/data topic

		if do_log_raw_data:
			rospy.loginfo(raw_data)
			if data["type"] != "velocity":
				continue
		else:
			if data["type"] != "velocity":
				continue

		altitude = data["altitude"]
		

		vx = data["vx"]
		vy = data["vy"]
		vz = data["vx"]
		vf = math.sqrt(vx**2+vy**2+vz**2)
		
		print(vf)
  
  
		velocity_twist_msg.header.stamp = rospy.Time()
		velocity_twist_msg.header.frame_id = "dvl_link"
		velocity_twist_msg.twist.twist.linear.x = vx
		velocity_twist_msg.twist.twist.linear.y = vy
		velocity_twist_msg.twist.twist.linear.z = vz


		velocity_pub.publish(vf)
		velocity_twist_pub.publish(velocity_twist_msg)
		altitude_pub.publish(altitude)
		#velocity_pub.publish(0)
                #velocity_twist_pub.publish(velocity_twist_msg)
                #altitude_pub.publish(0)
		#print("velocity = "+str(data['fom']))

		rate.sleep()

if __name__ == '__main__':
	global s, TCP_IP, TCP_PORT, do_log_raw_data
	rospy.init_node('a50_pub', anonymous=True)
	TCP_IP = rospy.get_param("~ip", "192.168.194.95")
	TCP_PORT = rospy.get_param("~port", 16171)
	do_log_raw_data = rospy.get_param("~do_log_raw_data", False)
	connect()
	try:
		publisher()
	except rospy.ROSInterruptException:
		s.close()
