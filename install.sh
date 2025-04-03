#!/bin/bash

echo "╔══╣ Setup: SOBIT Follower (STARTING) ╠══╗"


# Keep track of the current directory
DIR=`pwd`

echo -e "\e[34m\n Git Clone 2d_lidar_person_detection \e[m"
git clone -b humble-devel https://github.com/TeamSOBITS/2d_lidar_person_detection.git
cd 2d_lidar_person_detection/dr_spaam/
sudo python3 setup.py install

# Download default weight file from Google Drive
echo -e "\e[34m\n Downloading required weight file from Google Drive \e[m"
cd ../dr_spaam_ros/weights/
FILE_ID="1JfGzRotJSapktNjlcNZ_k4IJKwiRTQwa"
FILE_NAME="ckpt_jrdb_ann_ft_dr_spaam_e20.pth"

# Use wget to download from Google Drive
echo "Downloading ${FILE_NAME} from Google Drive..."
wget --no-check-certificate "https://docs.google.com/uc?export=download&id=${FILE_ID}" -O ${FILE_NAME}
if [ $? -ne 0 ]; then
    echo "Failed to download ${FILE_NAME} from Google Drive."
    exit 1
fi
echo "Successfully downloaded ${FILE_NAME}"

# Return to the colcon_ws/src directory
cd ~/colcon_ws/src

# Dowload required packages for SOBIT Follower
ros_packages=(
    "sobits_msgs" \
    "ssd_nodelet" \
    "sobit_edu" \
    "sobit_pro"
)

# Clone all packages
for ((i = 0; i < ${#ros_packages[@]}; i++)) {
    echo "Clonning: ${ros_packages[i]}"
    git clone -b humble-devel https://github.com/TeamSOBITS/${ros_packages[i]}.git

    # Check if install.sh exists in each package
    if [ -f ${ros_packages[i]}/install.sh ]; then
        echo "Running install.sh in ${ros_packages[i]}."
        cd ${ros_packages[i]}
        bash install.sh
        cd ..
    fi
}

# Download ROS packages
sudo apt-get update
sudo apt-get install -y \
    ros-$ROS_DISTRO-pointcloud-to-laserscan -y

echo "╚══╣ Setup: SOBIT Follower (FINISHED) ╠══╝"