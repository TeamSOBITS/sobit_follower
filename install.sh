#!/bin/sh

sudo apt update

echo -e "\e[34m\n Install ros-${ROS_DISTRO}-pointcloud-to-laserscan \e[m"
sudo apt install ros-${ROS_DISTRO}-pointcloud-to-laserscan -y

echo -e "\e[34m\n Git Clone 2d_lidar_person_detection \e[m"
git clone https://github.com/TeamSOBITS/2d_lidar_person_detection.git

cd ~/catkin_ws/src/

echo -e "\e[34m\n Git Clone ssd_nodelet \e[m"
git clone https://github.com/TeamSOBITS/ssd_nodelet.git

echo -e "\e[34m\n Git Clone sobit_common\e[m"
git clone https://github.com/TeamSOBITS/sobit_common.git

echo -e "\e[34m\n Git Clone SOBIT PRO \e[m"
git clone https://github.com/TeamSOBITS/sobit_pro.git

echo -e "\e[34m\n Git Clone scan2d_handler \e[m"
git clone https://github.com/TeamSOBITS/scan2d_handler.git

echo -e "\e[34m\n Git Clone virtual_multiple_sensor_publisher \e[m"
git clone https://github.com/TeamSOBITS/virtual_multiple_sensor_publisher.git

echo -e "\e[34m\n Git Clone following_control_methods \e[m"
git clone https://github.com/TeamSOBITS/following_control_methods.git

# echo -e "\e[34m\n Git Clone SOBIT PRO \e[m"
# git clone https://github.com/TeamSOBITS/sobit_pro.git

echo -e "\e[34m\n Install: Sobit Common (STARTING) \e[m"
sudo apt-get update
sudo apt-get install -y \
    ros-${ROS_DISTRO}-kobuki-* \
    ros-${ROS_DISTRO}-ecl-streams \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-joint-state-publisher* \
    ros-${ROS_DISTRO}-ros-control \
    ros-${ROS_DISTRO}-ros-controllers

sudo cp ~/catkin_ws/src/sobit_common/turtlebot2/turtlebot_simulator/turtlebot_gazebo/libgazebo_ros_kobuki.so /opt/ros/${ROS_DISTRO}/lib

# 関係ない
sudo apt-get install -y \
    ros-${ROS_DISTRO}-pcl-* \
    ros-${ROS_DISTRO}-openni2-*

echo -e "\e[34m\n Install: Sobit Common (FINISHED) \e[m"

cd

# Setting Sound configure
echo "pacmd load-module module-native-protocol-unix socket=/tmp/pulseaudio.socket &> /dev/null" >> ~/.bashrc
echo "#!bin/bash
touch /tmp/pulseaudio.client.conf
echo \"default-server = unix:/tmp/pulseaudio.socket \n
    # Prevent a server running in the container \n
    autospawn = no \n
    daemon-binary = /bin/true \n
    # Prevent the use of shared memory \n
    enable-shm = false\" >> /tmp/pulseaudio.client.conf" | sudo tee /etc/profile.d/sound_setup.sh
sudo bash /etc/profile.d/sound_setup.sh

# Seting dynamixel USB1 (SOBIT PRO arm_pantilt)
echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6015\", SYMLINK+=\"input/dynamixel1\", MODE=\"0666\"" | sudo tee /etc/udev/rules.d/dynamixel1.rules
# echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6015\", ATTRS{serial}==\"E143\", SYMLINK+=\"input/dongle\", MODE=\"0666\"" | sudo tee /etc/udev/rules.d/dongle.rules
# sudo /etc/init.d/udev reload

# Seting dynamixel USB2 (SOBIT PRO wheel)
echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6014\", SYMLINK+=\"input/dynamixel2\", MODE=\"0666\"" | sudo tee /etc/udev/rules.d/dynamixel2.rules
# echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6014\", ATTRS{serial}==\"E148\", SYMLINK+=\"input/dxhub\", MODE=\"0666\"" | sudo tee /etc/udev/rules.d/dxhub.rules
# sudo /etc/init.d/udev reload

# Seting ps4_joy_control USB
echo "KERNEL==\"uinput\", MODE=\"0666\"
    KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", ATTRS{idVendor}==\"054c\", ATTRS{idProduct}==\"05c4\", MODE=\"0666\"
    KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", KERNELS==\"0005:054C:05C4.*\", MODE=\"0666\"
    KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", ATTRS{idVendor}==\"054c\", ATTRS{idProduct}==\"09cc\", MODE=\"0666\"
    KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", KERNELS==\"0005:054C:09CC.*\", MODE=\"0666\"" | sudo tee /etc/udev/rules.d/50-ds4drv.rules
# sudo /etc/init.d/udev reload

# Seting azure_kinect USB
echo "# Bus 002 Device 116: ID 045e:097a Microsoft Corp.
    # Bus 001 Device 015: ID 045e:097b Microsoft Corp.
    # Bus 002 Device 118: ID 045e:097c Microsoft Corp.
    # Bus 002 Device 117: ID 045e:097d Microsoft Corp.
    # Bus 001 Device 016: ID 045e:097e Microsoft Corp.
    BUS!=\"usb\", ACTION!=\"add\", SUBSYSTEM!==\"usb_device\", GOTO=\"k4a_logic_rules_end\"
    ATTRS{idVendor}==\"045e\", ATTRS{idProduct}==\"097a\", MODE=\"0666\", GROUP=\"plugdev\"
    ATTRS{idVendor}==\"045e\", ATTRS{idProduct}==\"097b\", MODE=\"0666\", GROUP=\"plugdev\"
    ATTRS{idVendor}==\"045e\", ATTRS{idProduct}==\"097c\", MODE=\"0666\", GROUP=\"plugdev\"
    ATTRS{idVendor}==\"045e\", ATTRS{idProduct}==\"097d\", MODE=\"0666\", GROUP=\"plugdev\"
    ATTRS{idVendor}==\"045e\", ATTRS{idProduct}==\"097e\", MODE=\"0666\", GROUP=\"plugdev\"
    LABEL=\"k4a_logic_rules_end\"" | sudo tee /etc/udev/rules.d/99-k4a.rules
# sudo /etc/init.d/udev reload

# Set up arduino-nano rules
# echo "ATTRS{idVendor}==\"1a86\", ATTRS{idProduct}==\"7523\", MODE=\"0666\", RUN+=\"/bin/stty -F /dev/ttyUSB0\"" | sudo tee /etc/udev/rules.d/60-arduino-nano.rules
# sudo /etc/init.d/udev reload

# Set up kobuki rules
echo "# On precise, for some reason, USER and GROUP are getting ignored.
    # So setting mode = 0666 for now.
    SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6001\", ATTRS{serial}==\"kobuki*\", 
    ATTR{device/latency_timer}=\"1\", MODE:=\"0666\", GROUP:=\"dialout\", SYMLINK+=\"input/kobuki\", KERNEL==\"ttyUSB*\"
    # Bluetooth module (currently not supported and may have problems)
    # SUBSYSTEM==\"tty\", ATTRS{address}==\"00:00:00:41:48:22\", MODE:=\"0666\", GROUP:=\"dialout\", SYMLINK+=\"input/kobuki\"" | sudo tee /etc/udev/rules.d/57-kobuki.rules
# sudo /etc/init.d/udev reload

# Set up xtion rules
echo "# Make primesense device mount with writing permissions (default is read only for unknown devices)
    SUBSYSTEM==\"usb\", ATTR{idProduct}==\"0200\", ATTR{idVendor}==\"1d27\", MODE:=\"0666\", OWNER:=\"root\", GROUP:=\"video\"
    SUBSYSTEM==\"usb\", ATTR{idProduct}==\"0300\", ATTR{idVendor}==\"1d27\", MODE:=\"0666\", OWNER:=\"root\", GROUP:=\"video\"
    SUBSYSTEM==\"usb\", ATTR{idProduct}==\"0401\", ATTR{idVendor}==\"1d27\", MODE:=\"0666\", OWNER:=\"root\", GROUP:=\"video\"
    SUBSYSTEM==\"usb\", ATTR{idProduct}==\"0500\", ATTR{idVendor}==\"1d27\", MODE:=\"0666\", OWNER:=\"root\", GROUP:=\"video\"
    SUBSYSTEM==\"usb\", ATTR{idProduct}==\"0600\", ATTR{idVendor}==\"1d27\", MODE:=\"0666\", OWNER:=\"root\", GROUP:=\"video\"
    SUBSYSTEM==\"usb\", ATTR{idProduct}==\"0601\", ATTR{idVendor}==\"1d27\", MODE:=\"0666\", OWNER:=\"root\", GROUP:=\"video\"
    SUBSYSTEM==\"usb\", ATTR{idProduct}==\"0609\", ATTR{idVendor}==\"1d27\", MODE:=\"0666\", OWNER:=\"root\", GROUP:=\"video\"
    SUBSYSTEM==\"usb\", ATTR{idProduct}==\"1280\", ATTR{idVendor}==\"1d27\", MODE:=\"0666\", OWNER:=\"root\", GROUP:=\"video\"
    SUBSYSTEM==\"usb\", ATTR{idProduct}==\"2100\", ATTR{idVendor}==\"1d27\", MODE:=\"0666\", OWNER:=\"root\", GROUP:=\"video\"
    SUBSYSTEM==\"usb\", ATTR{idProduct}==\"2200\", ATTR{idVendor}==\"1d27\", MODE:=\"0666\", OWNER:=\"root\", GROUP:=\"video\"
    SUBSYSTEM==\"usb\", ATTR{idProduct}==\"f9db\", ATTR{idVendor}==\"1d27\", MODE:=\"0666\", OWNER:=\"root\", GROUP:=\"video\"" | sudo tee /etc/udev/rules.d/55-primesense-usb.rules
# sudo /etc/init.d/udev reload

sudo udevadm control --reload-rules
sudo udevadm trigger

# USB Reload
sudo /etc/init.d/udev reload

cd
cd ~/catkin_ws/src/sobit_follower/2d_lidar_person_detection/dr_spaam/
sudo python3 setup.py install

# cd ~/catkin_ws/
# catkin_make -j$[$(grep cpu.cores /proc/cpuinfo | sort -u | sed 's/[^0-9]//g') + 1] -DCMAKE_CXX_FLAGS=-O3
