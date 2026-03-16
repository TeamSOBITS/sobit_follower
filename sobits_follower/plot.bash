#!/bin/bash

ROSBAG_DIR=~/catkin_ws/src/sobits_follower/sobits_follower/rosbag
cd $ROSBAG_DIR

# rosbag file making csv
rosbag_files=( # if all rosbag files are to be processed, leave this array empty
    ours_A_windex_spray_bottle_2025-01-07-14-34-47
)
# Check if the array is empty
if [ ${#rosbag_files[@]} -eq 0 ]; then
    echo "No rosbag files manually specified. Fetching all .bag files..."
    rosbag_files=($(ls *.bag | sed 's/\.bag$//'))
fi


for ((i = 0; i < ${#rosbag_files[@]}; i++)) {
    echo "${rosbag_files[i]}"
    # path=~/catkin_ws/src/sobits_follower/sobits_follower/experimental_data/${rosbag_files[i]}
    path=~/catkin_ws/src/sobits_follower/sobits_follower/experimental_data/trial2_20250107/post/${rosbag_files[i]}

    python3 ~/catkin_ws/src/sobits_follower/sobits_follower/scripts/haku_plot.py \
        --following_position_csv_path "$path/following_position_${rosbag_files[i]}.csv" \
        --target_postion_odom_csv_path "$path/target_postion_odom_${rosbag_files[i]}.csv" \
        --odom_csv_path "$path/odom_${rosbag_files[i]}.csv" \
        --raw_cmd_vel_csv_path "$path/raw_cmd_vel_${rosbag_files[i]}.csv" \
        --odom_velocity_csv_path "$path/odom_velocity_${rosbag_files[i]}.csv" \
        --save_plot_path "$path/${rosbag_files[i]}" \
        --data_folder "${rosbag_files[i]}"
}
