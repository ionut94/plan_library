# PlanLib docker image
FROM kclplanning/rosplan:latest

SHELL ["/bin/bash", "-c"]
WORKDIR /root/ws

# Clone plan_lib
RUN git clone https://github.com/ionut94/plan_library.git src/plan_library


# Further dependencies
RUN apt install ros-melodic-rviz-visual-tools -qq -y &&\
	source devel/setup.bash &&\
    rosdep update &&\
    rosdep install --from-paths src/plan_library --ignore-src -q -r -y

# Build workspace
RUN catkin build --summarize --no-status

# Important to run this from the run_docker.sh script
ENTRYPOINT ["bash", "/root/ws/src/plan_library/experiments"]

