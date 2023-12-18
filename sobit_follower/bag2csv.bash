#!/bin/bash

cd ~/catkin_ws/src
# rosbag file making csv
rosbag_files=(
    # "rosbag_file_name" \
)

for ((i = 0; i < ${#rosbag_files[@]}; i++)) {
    cd ~/catkin_ws/src
    echo "${rosbag_files[i]}"
    rostopic echo -b ${rosbag_files[i]}.bag -p /sobit_follower/following_position/pose/position > following_position_${rosbag_files[i]}.csv
    rostopic echo -b ${rosbag_files[i]}.bag -p /sobit_follower/target_postion_odom/point > target_postion_odom_${rosbag_files[i]}.csv
    rostopic echo -b ${rosbag_files[i]}.bag -p /odom/pose/pose/position > odom_${rosbag_files[i]}.csv
    rostopic echo -b ${rosbag_files[i]}.bag -p /sobit_follower/velocity_smoother/raw_cmd_vel > raw_cmd_vel_${rosbag_files[i]}.csv
    rostopic echo -b ${rosbag_files[i]}.bag -p /cmd_vel_mux/input/teleop  > smooth_cmd_vel_${rosbag_files[i]}.csv
    rostopic echo -b ${rosbag_files[i]}.bag -p /odom/twist/twist> odom_velocity_${rosbag_files[i]}.csv
    path="sobit_follower/sobit_follower/experimental_data/${rosbag_files[i]}"
    mkdir $path
    mv following_position_${rosbag_files[i]}.csv $path
    mv target_postion_odom_${rosbag_files[i]}.csv $path
    mv odom_${rosbag_files[i]}.csv $path
    mv raw_cmd_vel_${rosbag_files[i]}.csv $path
    mv smooth_cmd_vel_${rosbag_files[i]}.csv $path
    mv odom_velocity_${rosbag_files[i]}.csv $path

    # echo "$path"
    python3 sobit_follower/sobit_follower/scripts/plot.py \
        --following_position_csv_path "$path/following_position_${rosbag_files[i]}.csv" \
        --target_postion_odom_csv_path "$path/target_postion_odom_${rosbag_files[i]}.csv" \
        --odom_csv_path "$path/odom_${rosbag_files[i]}.csv" \
        --raw_cmd_vel_csv_path "$path/raw_cmd_vel_${rosbag_files[i]}.csv" \
        --smooth_cmd_vel_csv_path "$path/smooth_cmd_vel_${rosbag_files[i]}.csv" \
        --odom_velocity_csv_path "$path/odom_velocity_${rosbag_files[i]}.csv" \
        --save_plot_path "$path/${rosbag_files[i]}"
}