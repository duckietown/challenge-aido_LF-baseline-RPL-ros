#!/bin/bash

source /environment.sh

source /opt/ros/noetic/setup.bash
source /code/catkin_ws/devel/setup.bash --extend
source /code/solution/devel/setup.bash --extend
source /code/submission_ws/devel/setup.bash --extend


set -eux

dt-exec-BG roscore

dt-exec-BG roslaunch --wait base_master base_master.launch
dt-exec-FG roslaunch --wait agent agent_node.launch || true
