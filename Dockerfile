FROM pytwb

RUN pip3 install matplotlib transforms3d sympy pyquaternion

RUN apt-get update && apt-get install -y --no-install-recommends \
 ros-humble-rmw-cyclonedds-cpp

# Use Cyclone DDS as middleware
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /usr/local/lib
RUN git clone https://github.com/RobotSpatialCognition/vector_map.git
WORKDIR /usr/local/lib/vector_map
RUN pip3 install -e .
WORKDIR /root
