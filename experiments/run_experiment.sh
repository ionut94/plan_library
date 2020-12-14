#!/bin/bash
# This is set to be run inside the docker container!

echo "RUNNING PLAN LIBRARY EXPERIMENT"
echo -e "PARAMS: $@ \n\n"

source /opt/ros/melodic/setup.bash
source devel/setup.bash

PROBLEMS_PATH="/root/ws/src/plan_library/common/benchmarks/office"

for INSTANCE in $(ls $PROBLEMS_PATH/problem_*); do
    sleep 7 && rosservice call /plan_library/execute_current_problem & # Wait for the nodes to launch, then trigger planning
    timeout 500 roslaunch plan_library test_plan_lib.launch $@ problem_path:=$PROBLEMS_PATH/$INSTANCE
    echo -e '\n\n\n\n\n'
done

for INSTANCE in $(ls $PROBLEMS_PATH/problem_*); do
    sleep 7 && rosservice call /plan_library/execute_current_problem & # Wait for the nodes to launch, then trigger planning
    timeout 500 roslaunch plan_library test_plan_lib.launch $@ problem_path:=$PROBLEMS_PATH/$INSTANCE
    echo -e '\n\n\n\n\n'
done


for INSTANCE in $(ls $PROBLEMS_PATH/problem_*); do
    sleep 7 && rosservice call /plan_library/execute_current_problem & # Wait for the nodes to launch, then trigger planning
    timeout 500 roslaunch plan_library test_plan_lib.launch $@ problem_path:=$PROBLEMS_PATH/$INSTANCE
    echo -e '\n\n\n\n\n'
done