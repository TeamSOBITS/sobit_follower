#!/usr/bin/env python
# coding: utf-8
import rospy
import sys
import re
import subprocess
import datetime
from geometry_msgs.msg import *
from subprocess import *
from std_msgs.msg import *
import tf

now = datetime.datetime.now()
fmt_name = "sample_{0:%Y年%m月%d日-%H時%M分}.txt".format(now)
path = '/home/iwami/catkin_ws/src/sobit_follower/txt/'
D = path + fmt_name
with open(D, mode='a') as f:

	def callback(data):
		f.write(str(data.linear.x))
		f.write("\t")
		f.write(str(data.angular.z))
		f.write("\t")

	if __name__ == '__main__':
		rospy.loginfo('ok')
		rospy.init_node('point_listener', anonymous=True)
		rospy.Subscriber("/data",Twist, callback)
		#rospy.Subscriber("/pub_vel",Twist, callback)
		rospy.spin()
		f.close()
