#!/bin/bash

roscore &
source /environment.sh
source /opt/ros/noetic/setup.bash
source /code/catkin_ws/devel/setup.bash
source /code/submission_ws/devel/setup.bash
python3 solution.py &
roslaunch --wait random_action random_action_node.launch
