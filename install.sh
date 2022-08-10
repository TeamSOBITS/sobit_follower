#!/bin/sh

sudo apt update 

echo -e "\e[34m\n Install ros-melodic-pointcloud-to-laserscan \e[m"
sudo apt install ros-melodic-pointcloud-to-laserscan -y

echo -e "\e[34m\n Install ros-melodic-yocs-velocity-smoother \e[m"
sudo apt install ros-melodic-yocs-velocity-smoother -y

cd ~/catkin_ws/src/
echo -e "\e[34m\n Git Clone pcl_handle \e[m"
git clone https://gitlab.com/TeamSOBITS/pcl_handle.git

echo -e "\e[34m\n Git Clone libsvm_catkin \e[m"
git clone https://gitlab.com/TeamSOBITS/sobit_libsvm.git

echo -e "\e[34m\n Git Clone svdd_lrf_leg_tracker \e[m"
git clone https://gitlab.com/TeamSOBITS/svdd_lrf_leg_tracker.git

echo -e "\e[34m\n Git Clone ssd_nodelet \e[m"
git clone https://gitlab.com/TeamSOBITS/ssd_nodelet.git

echo -e "\e[34m\n Git Clone SOBIT EDU \e[m"
git clone https://gitlab.com/TeamSOBITS/sobit_education.git

echo -e "\e[34m\n Git Clone SOBIT MINI \e[m"
git clone https://gitlab.com/TeamSOBITS/sobit_mini.git

# echo -e "\e[34m\n Git Clone SOBIT PRO \e[m"
# git clone https://gitlab.com/TeamSOBITS/sobit_pro.git

cd sobit_education/
bash sobit_setup.sh 

cd ..
catkin_make -j$[$(grep cpu.cores /proc/cpuinfo | sort -u | sed 's/[^0-9]//g') + 1] -DCMAKE_CXX_FLAGS=-O3
