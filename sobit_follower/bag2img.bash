#!/bin/bash

ROSBAG_DIR=~/catkin_ws/src/sobit_follower/sobit_follower/rosbag
cd $ROSBAG_DIR

# rosbag file making csv
rosbag_files=( # if all rosbag files are to be processed, leave this array empty
    ours_A_rope_2025-01-07-19-08-43
    ours_B_dice_2025-01-07-22-09-13
)
# Check if the array is empty
if [ ${#rosbag_files[@]} -eq 0 ]; then
    echo "No rosbag files manually specified. Fetching all .bag files..."
    rosbag_files=($(ls *.bag | sed 's/\.bag$//'))
fi

for ((i = 0; i < ${#rosbag_files[@]}; i++)) {
    # === Configuration ===
    INPUT_BAG=${rosbag_files[i]}.bag
    PLATE_BAG="plate_top_camera.bag"
    MASK_BAG="sam2_mask.bag"
    path=~/catkin_ws/src/sobit_follower/sobit_follower/experimental_data/${rosbag_files[i]}

    echo "Export data from ${rosbag_files[i]}"
    # Step 1: Filter ROS Bag Topics
    echo "Filtering /plate_top_camera/color/image_raw..."
    rosbag filter "$INPUT_BAG" "$PLATE_BAG" "topic == '/plate_top_camera/color/image_raw'"
    if [ $? -ne 0 ]; then
        echo "Failed to filter /plate_top_camera/color/image_raw"
        exit 1
    fi

    echo "Filtering /sam2_nontravelable_region_mask..."
    rosbag filter "$INPUT_BAG" "$MASK_BAG" "topic == '/sam2_nontravelable_region_mask'"
    if [ $? -ne 0 ]; then
        echo "Failed to filter /sam2_nontravelable_region_mask"
        exit 1
    fi

    python3 ~/catkin_ws/src/sobit_follower/sobit_follower/scripts/extract_SAM2_result.py \
        --plate_bag_path $PLATE_BAG \
        --mask_bag_path $MASK_BAG \
        --save_plot_path $path \
        
    rm plate_top_camera.bag sam2_mask.bag
}








