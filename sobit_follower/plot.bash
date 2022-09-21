#!/bin/

dir_name=$1
plot_path="experimental_data/$dir_name"
echo "plot_path = " $plot_path

# echo "$plot_path"
python3 scripts/plot.py \
    --following_position_csv_path "$plot_path/following_position_$dir_name.csv" \
    --target_postion_odom_csv_path "$plot_path/target_postion_odom_$dir_name.csv" \
    --odom_csv_path "$plot_path/odom_$dir_name.csv" \
    --raw_cmd_vel_csv_path "$plot_path/raw_cmd_vel_$dir_name.csv" \
    --smooth_cmd_vel_csv_path "$plot_path/smooth_cmd_vel_$dir_name.csv" \
    --odom_velocity_csv_path "$plot_path/odom_velocity_$dir_name.csv" \
    --save_plot_path "$plot_path/$dir_name"