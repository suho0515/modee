FROM osrf/ros:noetic-desktop-full

RUN /bin/bash -c "apt-get update &&\
    apt-get install -y git &&\
    apt-get install -y ros-noetic-usb-cam &&\
    source /opt/ros/noetic/setup.bash &&\
    mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src &&\
    cd ~/catkin_ws &&\
    catkin_make &&\
    source devel/setup.bash"