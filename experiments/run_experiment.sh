#!/bin/bash
# This is set to be run inside the docker container!

echo "RUNNING PLAN LIBRARY EXPERIMENT"
echo -e "PARAMS: $@ \n\n"

source /opt/ros/melodic/setup.bash
source devel/setup.bash


roslaunch plan_library test_plan_lib.launch $@  # To set: object file and initial state

