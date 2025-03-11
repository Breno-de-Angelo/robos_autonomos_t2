FROM osrf/ros:humble-desktop-full

# Install new gazebo
RUN apt-get update && apt-get install -y wget gnupg python3-vcstool && \
    wget -O - http://packages.osrfoundation.org/gazebo.key | gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && apt-get install -y ignition-fortress

# Download clearpath simulator and install dependencies
RUN mkdir /root/clearpath_ws/src -p && \
    cd /root/clearpath_ws/src && \
    git clone https://github.com/clearpathrobotics/clearpath_simulator.git -b humble && \
    cd /root/clearpath_ws/src/clearpath_simulator && \
    vcs import < dependencies.repos && \
    cd ../.. && \
    rosdep update && \
    rosdep install -r --from-paths src -i -y

# Build clearpath simulator
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /root/clearpath_ws && \
    colcon build --symlink-install"

# Create the clearpath directory and copy the robot.yaml into it
RUN mkdir -p /root/clearpath
COPY robot.yaml /root/clearpath/

WORKDIR /root/clearpath_ws

