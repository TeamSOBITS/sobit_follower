#!/bin/bash

ROSBAG_DIR=~/catkin_ws/src/sobit_follower/sobit_follower/rosbag
cd $ROSBAG_DIR

# rosbag file making csv
rosbag_files=( # if all rosbag files are to be processed, leave this array empty
    # rosbagname
)
# Check if the array is empty
if [ ${#rosbag_files[@]} -eq 0 ]; then
    echo "No rosbag files manually specified. Fetching all .bag files..."
    rosbag_files=($(ls *.bag | sed 's/\.bag$//'))
fi


for ((i = 0; i < ${#rosbag_files[@]}; i++)) {
    path=~/catkin_ws/src/sobit_follower/sobit_follower/experimental_data/${rosbag_files[i]}
    
    # Check if the folder already exists
    if [ -d "$path" ]; then
        echo "Skipping ${rosbag_files[i]}: Data already exported."
        continue
    fi
    
    # Export data from each rosbag
    echo "Export data from ${rosbag_files[i]}"
    rostopic echo -b ${rosbag_files[i]}.bag -p /sobit_follower/following_position/pose/position > following_position_${rosbag_files[i]}.csv
    rostopic echo -b ${rosbag_files[i]}.bag -p /sobit_follower/target_postion_odom/point > target_postion_odom_${rosbag_files[i]}.csv
    rostopic echo -b ${rosbag_files[i]}.bag -p /odom/pose/pose/position > odom_${rosbag_files[i]}.csv
    rostopic echo -b ${rosbag_files[i]}.bag -p /mobile_base/commands/velocity > raw_cmd_vel_${rosbag_files[i]}.csv
    rostopic echo -b ${rosbag_files[i]}.bag -p /odom/twist/twist> odom_velocity_${rosbag_files[i]}.csv
    
    # Create a folder for experimental data
    mkdir -p $path
    
    # Move CSV files into the designated folder
    mv following_position_${rosbag_files[i]}.csv $path
    mv target_postion_odom_${rosbag_files[i]}.csv $path
    mv odom_${rosbag_files[i]}.csv $path
    mv raw_cmd_vel_${rosbag_files[i]}.csv $path
    mv odom_velocity_${rosbag_files[i]}.csv $path
    
    # Generate plots using the Python script
    python3 ~/catkin_ws/src/sobit_follower/sobit_follower/scripts/haku_plot.py \
        --following_position_csv_path "$path/following_position_${rosbag_files[i]}.csv" \
        --target_postion_odom_csv_path "$path/target_postion_odom_${rosbag_files[i]}.csv" \
        --odom_csv_path "$path/odom_${rosbag_files[i]}.csv" \
        --raw_cmd_vel_csv_path "$path/raw_cmd_vel_${rosbag_files[i]}.csv" \
        --odom_velocity_csv_path "$path/odom_velocity_${rosbag_files[i]}.csv" \
        --save_plot_path "$path/${rosbag_files[i]}" \
        --data_folder "${rosbag_files[i]}"
}
