#!/usr/bin/env python

import commands as cd

import rospy
from std_msgs.msg import Float32

def talker():
	pub = rospy.Publisher('SensorData', Float32, queue_size=10)
	rospy.init_node('talker_sensor', anonymous=True)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		sensordata = float(cd.getoutput(
			'cat /sys/class/thermal/thermal_zone0/temp'))/1000
		hello_str = "Sensor data %f" % sensordata
		rospy.loginfo(hello_str)
		pub.publish(sensordata)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
