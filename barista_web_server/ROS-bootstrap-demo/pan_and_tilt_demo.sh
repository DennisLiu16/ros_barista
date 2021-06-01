#!/bin/bash
cd /home/rdaneel/ros_playground/pan_and_tilt_morpheuschair_ws
source devel/setup.bash
roslaunch pan_and_tilt_description main_sim.launch
rosrun web_video_server web_video_server
roslaunch rosbridge_server rosbridge_websocket.launch
roslaunch barista_systems loadfood_start_server.launch
# OPEN the LIVE server
# Basic example
open http://127.0.0.1:5500/index.html
# Example with services
open http://127.0.0.1:5500/index_2.html