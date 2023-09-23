#!/bin/bash

cd ~/catkin_ws/src
# csvを作るrosbagファイル
rosbag_files=(
	# 上のパスに存在するbag名を指定する
	"test_2023-09-23-17-39-22"
	
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
    # path="sobit_follower/sobit_follower/experimental_data/${rosbag_files[i]}"
    path="sobit_follow/sobit_follower/sobit_follower/experimental_data/${rosbag_files[i]}"
    mkdir $path
    bag_path="sobit_follow/sobit_follower/sobit_follower/experimental_data/${rosbag_files[i]}/rosbag"
    mkdir $bag_path
    plot_path="sobit_follow/sobit_follower/sobit_follower/experimental_data/${rosbag_files[i]}/plot"
    mkdir $plot_path
    csv_path="sobit_follow/sobit_follower/sobit_follower/experimental_data/${rosbag_files[i]}/csv"
    mkdir $csv_path

    mv following_position_${rosbag_files[i]}.csv $csv_path
    mv target_postion_odom_${rosbag_files[i]}.csv $csv_path
    mv odom_${rosbag_files[i]}.csv $csv_path
    mv raw_cmd_vel_${rosbag_files[i]}.csv $csv_path
    mv smooth_cmd_vel_${rosbag_files[i]}.csv $csv_path
    mv odom_velocity_${rosbag_files[i]}.csv $csv_path
    mv ${rosbag_files[i]}.bag $bag_path

    # echo "$path"
    python3 sobit_follow/sobit_follower/sobit_follower/scripts/plot_suzuki.py \
        --following_position_csv_path "$csv_path/following_position_${rosbag_files[i]}.csv" \
        --target_postion_odom_csv_path "$csv_path/target_postion_odom_${rosbag_files[i]}.csv" \
        --odom_csv_path "$csv_path/odom_${rosbag_files[i]}.csv" \
        --raw_cmd_vel_csv_path "$csv_path/raw_cmd_vel_${rosbag_files[i]}.csv" \
        --smooth_cmd_vel_csv_path "$csv_path/smooth_cmd_vel_${rosbag_files[i]}.csv" \
        --odom_velocity_csv_path "$csv_path/odom_velocity_${rosbag_files[i]}.csv" \
        --save_plot_path "$plot_path/${rosbag_files[i]}"
}
