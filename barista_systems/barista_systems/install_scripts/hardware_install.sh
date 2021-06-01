#!/usr/bin/env bash

RED='\e[41m'
GREEN='\e[42m'
RESET='\033[0m'

function echo_green {
    echo -e "\n${GREEN} $1 ${RESET}\n"
}

function echo_red {
    echo -e "\n${RED} $1 ${RESET}\n"
}

function install_ros {
	echo_green " Installing ROS Melodic"
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    sudo apt update
    sudo apt install ros-melodic-desktop-full -y
    sudo rosdep init
    rosdep update
    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
    printenv | grep ROS
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make
    source devel/setup.bash
	echo_green " Installing ROS Melodic...DONE"
}


function install_barista_basics {
    echo_green " Installing barista basics"
    sudo apt install python-pip -y
    sudo apt install terminator -y
    sudo apt install git -y
    sudo apt install vim -y
    sudo pat install simplescreenrecorder -y
    echo_green " Installing barista basics...DONE"
}

function install_barista_dependencies {
	echo_green " Installing barista dependencies"
    # Python modules

    # ROS packages
    sudo apt-get install ros-melodic-map-server -y
    sudo apt-get install ros-melodic-amcl -y
    sudo apt-get install ros-melodic-gmapping -y
    sudo apt-get install ros-melodic-joy -y
    sudo apt-get install ros-melodic-pid -y
    sudo apt-get install ros-melodic-move-base -y
    sudo apt-get install ros-melodic-dwa-local-planner -y
    sudo apt-get install ros-melodic-yocs-velocity-smoother

    # Needed for the droid speak package
    sudo pip install pygame

	echo_green " Installing barista dependencies...DONE"
}

function install_barista_code {
	echo_green " Installing barista dependencies"
	# We install dependencies needed for compiling this

	install_barista_dependencies
    # Python modules
    cd ~/catkin_ws/src
    git clone https://bitbucket.org/theconstructcore/barista.git

    # We want the barista version for the moment
    rm -rf kobuki/kobuki_description

    git clone https://bitbucket.org/theconstructcore/barista_systems.git
    git clone https://bitbucket.org/theconstructcore/barista_web_server.git

    git clone https://github.com/AriYu/ros_waypoint_generator.git
    git clone https://gitlab.com/easymov/droidspeak.git

    git clone https://bitbucket.org/theconstructcore/turtlebot.git
    cd turtlebot
    git checkout kinetic-gazebo9
    rm -rf turtlebot/kobuki_description turtlebot/turtlebot turtlebot/turtlebot_description turtlebot/turtlebot_teleop
    cd ~/catkin_ws/src

    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    echo_green " Installing barista dependencies...DONE"
}

function install_arduino_dependencies {
	echo_green " Installing ARDUINO dependencies"

	echo_green " Installing ARDUINO IDE"
    mkdir ~/Arduino
    cd ~/Arduino
    # https://tttapa.github.io/Pages/Ubuntu/Software-Installation/Arduino/Arduino-IDE.html
    wget https://downloads.arduino.cc/arduino-1.8.10-linux64.tar.xz
    tar -xf arduino-1.8.10-linux64.tar.xz
    cd arduino*/
    sudo ./install.sh
    rm ../arduino-1.8.10.tar.xz

    echo_green " Installing ARDUINO ROS dependencies"
    sudo apt-get install ros-melodic-rosserial-arduino -y
    sudo apt-get install ros-melodic-rosserial -y
	echo_green " Installing ARDUINO dependencies...DONE"
}

function install_kobuki_dependencies {
	echo_green " Installing Kobuki"
    cd ~/catkin_ws/src
    git clone https://github.com/yujinrobot/kobuki.git
    cd kobuki
    git checkout melodic

	sudo apt-get install ros-melodic-ecl
	sudo apt-get install ros-melodic-kobuki-msgs
	sudo apt-get install ros-melodic-yocs-controllers
	sudo apt-get install ros-melodic-kobuki-dock-drive
	sudo apt-get install ros-melodic-kobuki-driver
	sudo apt-get install ros-melodic-kobuki-ftdi

	cd ~/catkin_ws
	catkin_make
	source devel/setup.bash
	rospack profile

	sudo usermod -a -G dialout $USER
	rosrun kobuki_ftdi create_udev_rules

    # Test: In a first shell
    #$ . /opt/ros/kinetic/setup.bash
    #$ roslaunch kobuki_node minimal.launch --screen
    # In a second shell
    #$ . /opt/ros/kinetic/setup.bash
    #$ roslaunch kobuki_keyop keyop.launch --screen
	echo_green " Installing Kobuki...DONE"
}


function install_hokuyo{
	#test

    # for udev hokuyo Source
    # http://www.youbot-store.com/wiki/index.php/Hokuyo_URG-04LX-UG01
    cd ~/catkin_ws/src
    git clone https://github.com/ros-drivers/hokuyo_node.git
    sudo apt-get install ros-melodic-driver-base
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    rospack profile
    roscore
    # to know the USB port ubplug an dplug while executing:
    # tail -f /var/log/syslog
    #make it accesible in that session
    # rosrun hokuyo_node getID /dev/ttyACM0

    # This sets the ARDUINO and Hokuyo usb permissions permanently
    # like the kobuki does
    ls -l /dev/ttyACM0
    rosrun barista_systems create_udev_rules
    ls -l /dev/ttyACM0
    #Bus 002 Device 025: ID 15d1:0000
    #  idVendor           0x15d1
    #  idProduct          0x0000
    # Test:
    # roscore
    # rosrun hokuyo_node hokuyo_node
    # rosrun rviz rviz, open, hokuyo rviz config in barista_systems/rviz/hokuyo

}

function main {

    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
    cd ${SCRIPT_DIR}
    set -e
    #install_barista_basics
    #install_ros
    #install_barista_code
    install_kobuki_dependencies
    install_barista_dependencies
    set +e
}


main
