FROM ros:humble
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends vim xterm less python3-pip ros-humble-navigation2 ros-humble-py-trees ros-humble-py-trees-ros python3-opencv libopencv-contrib-dev python3-tk libqt5svg5-dev ros-humble-navigation2 ros-humble-rmw-cyclonedds-cpp

RUN pip3 install matplotlib transforms3d pyquaternion

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /usr/local/lib
RUN git clone https://github.com/momoiorg-repository/pytwb.git
WORKDIR /usr/local/lib/pytwb
RUN source /opt/ros/humble/setup.bash && pip3 install -e .

WORKDIR /usr/local/lib
RUN git clone https://github.com/RobotSpatialCognition/vector_map.git
WORKDIR /usr/local/lib/vector_map
RUN pip3 install -e .

WORKDIR /root
RUN mkdir pytwb_ws
# COPY ./pytwb_ws ./pytwb_ws
RUN echo "source /opt/ros/humble/setup.bash" >> .bashrc
