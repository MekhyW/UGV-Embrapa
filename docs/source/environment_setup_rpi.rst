Environment Setup - Raspberry Pi
================================

This section describes the steps to prepare the development environment for the UGV project, including the installation of the operating system, ROS 2 Humble, and the configuration of the Raspberry Pi for control.

Operating System Installation
-----------------------------

1. Download Ubuntu Server Image:
   
   - Go to the `official Ubuntu Server website <https://ubuntu.com/download/server>`_ and download the latest image.
   - Use software like Raspberry Pi Imager to save the image to the SD card.

2. Initial Setup:
   
   - Insert the SD card into the Raspberry Pi and turn on the device.
   - Configure Wi-Fi network and other basic preferences during startup.
   - Update the operating system:

     .. code-block:: sh

        sudo apt update
        sudo apt upgrade

Configuring Raspberry Pi as Server and SSH Communication
--------------------------------------------------------

1. Server Mode:
   
   - Server configuration allows Raspberry Pi to be accessed remotely.
   - Connect to the server using SSH:

     .. code-block:: sh

        ssh embrapa@10.42.0.1

ROS Humble Installation
-----------------------

1. Add ROS Repository Key:

   .. code-block:: sh

      sudo apt install software-properties-common
      sudo add-apt-repository universe
      sudo apt update
      sudo apt install curl
      curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

2. Add ROS Repository:

   .. code-block:: sh

      sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
      sudo apt update
      sudo apt install ros-humble-desktop

3. Configure ROS Environment:
   
   Add the following lines to the ``~/.bashrc`` file:

   .. code-block:: sh

      source /opt/ros/humble/setup.bash

Manual Rover Bringup
--------------------

To manually start the rover, run the following commands in the terminal:

1. Make sure the program is not already running:

   .. code-block:: bash

      sudo systemctl stop osr_startup

2. Execute the bringup command:

   .. code-block:: bash

      ros2 launch osr_bringup osr_launch.py

Automatic Bringup with Launch Script
------------------------------------

To automate the bringup and eliminate the need for SSH access, configure the Raspberry Pi to launch the rover code automatically:

1. Navigate to the ``init_scripts`` folder:

   .. code-block:: bash

      cd ~/osr_ws/src/osr-rover-code/init_scripts

2. Create symbolic links:

   .. code-block:: bash

      sudo ln -s $(pwd)/launch_osr.sh /usr/local/bin/launch_osr.sh
      sudo ln -s $(pwd)/osr_paths.sh /usr/local/bin/osr_paths.sh

3. Copy the service file to the systemd services directory:

   .. code-block:: bash

      sudo cp osr_startup.service /etc/systemd/system/osr_startup.service

4. Adjust service file permissions:

   .. code-block:: bash

      sudo chmod 644 /etc/systemd/system/osr_startup.service

Image Acquisition Setup
-----------------------

1. Installation of Required Libraries:

   .. code-block:: sh

      sudo apt install ros-humble-cv-bridge
      sudo apt install ros-humble-image-transport

2. Camera Configuration:
   
   - Connect Intel RealSense D435i and Logitech C920 HD PRO cameras to Raspberry Pi.
   - Use the following commands to check and adjust settings:

     .. code-block:: sh

        rs-enumerate-devices
        v4l2-ctl --list-devices

3. Publishing and Viewing Images:
   
   - Run ROS nodes to capture and publish images:

     .. code-block:: sh

        ros2 run realsense2_camera realsense2_camera_node
        ros2 run image_transport republish raw in:=/camera/color/image_raw out:=/image_raw

   - Another way to start capturing images:

     .. code-block:: sh

        systemctl restart start_turtle.service

   - To view images use Rviz2 or rqt_image_view:

     .. code-block:: sh

        ros2 run rqt_image_view rqt_image_view
        # or
        rviz2

This completes the environment setup for the UGV-Embrapa project. Make sure to follow these steps carefully to ensure a proper development environment.

