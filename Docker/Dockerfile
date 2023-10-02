FROM osrf/ros:humble-desktop-full
ARG ROS_DISTRO=humble

RUN apt-get update 
RUN apt-get -y install xauth \
                       sudo

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt-get -y install ros-${ROS_DISTRO}-turtlebot3-gazebo

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bash_profile
RUN mkdir ~/ros2_ws
# CMD ["rviz2"]
# CMD ["/bin/bash","-c","rviz2"]