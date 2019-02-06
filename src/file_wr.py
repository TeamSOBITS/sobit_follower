#!/usr/bin/env python
# coding: utf-8
import rospy
import cv2
import sys
import re
import subprocess
import datetime
from geometry_msgs.msg import *
from subprocess import *
from std_msgs.msg import *
import tf
from sobit_follower.msg import point_states
past_flag = True
now = datetime.datetime.now()
fmt_name = "sample_{0:%Y年%m月%d日-%H時%M分}.txt".format(now)
path = '/home/iwami/catkin_ws/src/sobit_follower/txt/'
D = path + fmt_name
with open(D, mode='a') as f:
	#f.write(data.robot_point.x," ",data.robot_point.y," ",data.target_point.point.x," ",data.target_point.point.y)

	def callback(data):
		global past_flag
	#end_time = datetime.datetime.now() + datetime.timedelta(10.0)
	#start_time = datetime.datetime.now()
		#if data.pre_flag == True and past_flag == False:
		#	f.write("pre_start")
		#	f.write("\n")
		#elif data.pre_flag == False and past_flag == True:
		#	f.write("pre_end")
		#	f.write("\n")
		f.write(str(data.robot_point.x))
		f.write("\t")
		f.write(str(data.robot_point.y))
		f.write("\t")
		f.write(str(data.target_point.point.x))
		f.write("\t")
		f.write(str(data.target_point.point.y))
		f.write("\n")

		past_flag = data.pre_flag
		
		#a = 10.0
		#b = 8
		#c = 9
		#d = 4
		#f.write(str(a))
		#f.write("\t")
		#f.write(str(b))
		#f.write("\t")
		#f.write(str(c))
		#f.write("\t")
		#f.write(str(d))
		#f.write("\n")

	if __name__ == '__main__':
		rospy.loginfo('ok')
		rospy.init_node('point_listener', anonymous=True)
		rospy.Subscriber("/pub_point_states",point_states, callback)
		#rospy.Subscriber("/pub_vel",Twist, callback)
		rospy.spin()
		f.close()
	#
