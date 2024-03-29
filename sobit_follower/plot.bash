#!/bin/bash

cd ~/catkin_ws/src
# csvを作るrosbagファイル
rosbag_files=(
    "murakami_0922_01" \
    "murakami_0922_02" \
    "murakami_0922_03"
)

for ((i = 0; i < ${#rosbag_files[@]}; i++)) {
    cd ~/catkin_ws/src
    echo "${rosbag_files[i]}"
    path="sobit_follower/sobit_follower/experimental_data/${rosbag_files[i]}"
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