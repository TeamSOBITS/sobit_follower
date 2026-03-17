#!/bin/bash

rosbag_file="sobits_follower"

if [ $# = 0 ]; then
    echo "rosbag file name = " $rosbag_file
else
    echo "rosbag file name = " $1
    rosbag_file=$1
fi

cd ~/catkin_ws/src/sobits_follower/sobits_follower/rosbag
rosbag record   /sobits_follower/multiple_sensor_person_tracking/following_position  \
                /sobits_follower/multiple_sensor_person_tracking/target_postion_odom  \
                /odom \
                /mobile_base/commands/velocity \
                /plate_top_camera/color/image_raw \
                /sam2_nontravelable_region_mask \
                -o $rosbag_file
