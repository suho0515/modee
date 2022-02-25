FROM osrf/ros:noetic-desktop-full

# Install usb-cam package
RUN /bin/bash -c "apt-get update &&\
    apt-get install -y git &&\
    apt-get install -y ros-noetic-usb-cam &&\
    source /opt/ros/noetic/setup.bash &&\
    mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src &&\
    cd ~/catkin_ws &&\
    catkin_make &&\
    source devel/setup.bash"

# Install Python
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
    apt-get install -y python3-pip"

# Install Python Package for ocr
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&\
    apt-get install -y tesseract-ocr tesseract-ocr-script-hang tesseract-ocr-script-hang-vert &&\
    pip3 install pytesseract"
