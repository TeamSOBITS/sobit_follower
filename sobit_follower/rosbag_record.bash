#!/bin/bash

rosbag_file="sobit_follower"

if [ $# = 0 ]; then
    echo "rosbag file name = " $rosbag_file
else
    echo "rosbag file name = " $1
    rosbag_file=$1
fi

cd ~/catkin_ws/src
rosbag record   /sobit_follower/following_position  \
                /sobit_follower/target_postion_odom  \
                /odom /cmd_vel_mux/input/teleop  \
                /sobit_follower/velocity_smoother/raw_cmd_vel \
                /person_id_follow_nodelet/target \
                /person_id_follow_nodelet/person_id_img \
                /ssd_object_detect/detect_result \
                -o $rosbag_file