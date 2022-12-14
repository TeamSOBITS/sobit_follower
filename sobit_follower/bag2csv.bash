#!/bin/bash

cd ~/catkin_ws/src
# csvを作るrosbagファイル
rosbag_files=(
    # "gonbe_1_2022-10-18-20-18-20" \
    # "gonbe_2_2022-10-18-20-19-38" \
    # "gonbe_3_2022-10-18-20-21-09" \
    # "hiro_1_2022-10-18-19-50-48" \
    # "hiro_2_2022-10-18-19-52-05" \
    # "hiro_3_2022-10-18-19-53-13" \
    "ikeda_1_2022-10-18-19-16-04" \
    # "ikeda_2_2022-10-18-19-16-56" \
    # "ikeda_3_2022-10-18-19-18-09" \
    # "ikeuchi_1_2022-10-18-19-34-45" \
    # "ikeuchi_2_2022-10-18-19-36-03" \
    # "ikeuchi_3_2022-10-18-19-37-19" \
    # "kani_1_2022-10-18-20-01-34" \
    # "kani_2_2022-10-18-20-03-45" \
    # "kani_3_2022-10-18-20-04-56" \
    # "koshiro_1_2022-10-18-20-06-59" \
    # "koshiro_2_2022-10-18-20-08-26" \
    # "koshiro_3_2022-10-18-20-09-54" \
    # "mukogawa_1_2022-10-18-19-46-45" \
    # "mukogawa_2_2022-10-18-19-48-00" \
    # "mukogawa_3_2022-10-18-19-49-24" \
    # "sakamot_1_2022-10-18-19-19-59" \
    # "sakamot_2_2022-10-18-19-21-23" \
    # "sakamot_3_2022-10-18-19-22-37" \
    # "taichi_1_2022-10-18-19-31-03" \
    # "taichi_2_2022-10-18-19-32-03" \
    # "taichi_3_2022-10-18-19-33-16" \
    "tsuru_1_2022-10-18-20-13-46"
    # "tsuru_2_2022-10-18-20-15-04" \
    # "tsuru_3_2022-10-18-20-16-31"
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