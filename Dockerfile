FROM osrf/ros:humble-desktop-full

WORKDIR /root

# Create the clearpath directory and copy the robot.yaml into it
RUN mkdir -p /root/clearpath
COPY robot.yaml /root/clearpath/
COPY ./requirements.txt /root/requirements.txt 

# Install new gazebo
RUN apt-get update && apt-get install -y wget gnupg python3-vcstool python3-pip ros-humble-clearpath-nav2-demos && \
    wget -O - http://packages.osrfoundation.org/gazebo.key | gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && apt-get install -y ignition-fortress

# Download clearpath simulator and install dependencies
RUN mkdir /root/clearpath_ws/src -p && \
    cd /root/clearpath_ws/src && \
    git clone --depth 1 https://github.com/clearpathrobotics/clearpath_simulator.git -b humble && \
    cd /root/clearpath_ws/src/clearpath_simulator && \
    vcs import < dependencies.repos && \
    cd ../.. && \
    rosdep update && \
    rosdep install -r --from-paths src -i -y

# Download explore-lite (no dependencies in dependencies.repos)
RUN mkdir -p /root/explore_lite_ws/src && \
    cd /root/explore_lite_ws/src && \
    git clone --depth 1 https://github.com/robo-friends/m-explore-ros2.git && \
    cd /root/explore_lite_ws && \
    rosdep update && \
    rosdep install -r --from-paths src -i -y

# Build and Source clearpath simulator
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
                  cd /root/clearpath_ws && \
                  colcon build --symlink-install 2> /dev/null && \
                  source /root/clearpath_ws/install/setup.bash"

#Build and Source explore-lite 
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
                  cd /root/explore_lite_ws && \
                  colcon build --symlink-install 2>/dev/null && \
                  source /root/explore_lite_ws/install/setup.bash"
    
# Add ROS2 Setup to bashrc (Non-Interactive Mode Shell)
# Source clearpath to bashrc
# Source explore_lite to bashrc
# Alias srcinst to make it easy to install our pkg
# Get Recognition Model
# Install requirements via pip 

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/clearpath_ws/install/setup.bash" >> /root/.bashrc && \
    echo "source /root/explore_lite_ws/install/setup.bash" >> /root/.bashrc && \
    echo "alias srcinst='source /root/robos_autonomos_t2/install/setup.bash'" >> /root/.bashrc && \
    echo "alias cbs='colcon build --symlink-install'" >> /root/.bashrc && \
    /bin/bash -c "wget -O pose_landmarker.task -q https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_heavy/float16/1/pose_landmarker_heavy.task" && \
    pip install -r /root/requirements.txt

