#!/usr/bin/env python
#coding: utf-8

from time import sleep
import subprocess

time = 3
cmd = "gnome-terminal --geometry=80x40+100+100 -x roslaunch turtlebot_edu minimal.launch "
subprocess.Popen(cmd,shell=True)
sleep(time + 5)
