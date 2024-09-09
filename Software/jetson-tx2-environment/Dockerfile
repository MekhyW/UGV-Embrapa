#
# Dockerfile for installing additional packages from source in ROS2
#
FROM dustynv/ros:humble-ros-base-l4t-r32.7.1

USER root

ARG ROS_WORKSPACE=${ROS_ROOT}
    
# Copy all .sh files into the container
COPY ./*.sh /scripts/

# Make sure all shell scripts are executable
RUN chmod +x /scripts/*.sh

#RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null \
#    && echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' | tee /etc/apt/sources.list.d/kitware.list >/dev/null
#
# Install additional ROS packages using the desired script
RUN /scripts/ros2_install.sh rtabmap_ros realsense2_camera https://github.com/MekhyW/UGV-Embrapa.git


