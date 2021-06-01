#!/bin/bash
sudo apt update
sudo apt remove ros-kinetic-rosbridge-suite
sudo apt-get remove python-tornado
sudo pip uninstall tornado
sudo apt install python-pip ros-kinetic-rosbridge-suite ros-kinetic-web-video-server nginx