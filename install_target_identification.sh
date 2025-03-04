#!/bin/bash

echo "╔══╣ Setup: SOBIT Follower(Target Identification) (STARTING) ╠══╗"


# Keep track of the current directory
DIR=`pwd`

# Return to the catkin_ws/src directory
cd ~/catkin_ws/src

# Dowload required packages for SOBIT Follower
ros_packages=(
    "yolov10_ros" \
    "bbox_to_tf" \
    "monocular_person_following" \
    "ccf_person_identification" \
    "MPF_GRR_SLT"
)

# Clone all packages
for ((i = 0; i < ${#ros_packages[@]}; i++)) {
    echo "Clonning: ${ros_packages[i]}"
    git clone https://github.com/TeamSOBITS/${ros_packages[i]}.git

    # Check if install.sh exists in each package
    if [ -f ${ros_packages[i]}/install.sh ]; then
        echo "Running install.sh in ${ros_packages[i]}."
        cd ${ros_packages[i]}
        bash install.sh
        cd ..
    fi
}

# Move into the MPF_GRR_SLT package directory
cd MPF_GRR_SLT

# Check if requirements.txt exists and install dependencies
if [ -f requirements.txt ]; then
    echo "Installing Python dependencies from requirements.txt..."
    pip3 install -r requirements.txt
else
    echo "No requirements.txt found, skipping dependency installation."
fi

# Check if ros_numpy exists and install it
if [ -d ros_numpy ]; then
    echo "Installing ros_numpy..."
    cd ros_numpy
    python3 setup.py install
    cd ..
else
    echo "ros_numpy directory not found, skipping installation."
fi

cd ..

echo "Installation completed."

# Download ROS packages
sudo apt-get update

echo "╚══╣ Setup: SOBIT Follower(Target Identification) (FINISHED) ╠══╝"