Environment Setup - Dell Optiplex
=================================

This section describes the steps to prepare the development environment for the UGV project on the Dell Optiplex, which is the computing platform for the SLAM/autonomous navigation module. The Optiplex communicates with the Raspberry Pi over Ethernet during operation.

1. Install ROS 2 Humble and Required Packages
---------------------------------------------

Run the following commands to install necessary packages:

.. code-block:: bash

   sudo apt install ros-humble-desktop
   sudo apt install ros-humble-librealsense2*
   sudo apt install ros-humble-realsense2-*
   sudo apt install ros-humble-rtabmap-ros
   sudo apt install ros-humble-imu-tools
   sudo apt install ros-humble-control-msgs

2. Create a Colcon Workspace
----------------------------

Set up a Colcon Workspace to manage ROS packages without official Rosdistro binaries:

.. code-block:: bash

   mkdir -p ~/osr_ws/src
   cd ~/osr_ws/src

3. Clone Required Repositories
------------------------------

Download the following repositories into the `src` folder:

.. code-block:: bash

   git clone https://github.com/ROBOTIS-GIT/ld08_driver.git
   git clone https://github.com/your-username/UGV-Embrapa.git

4. Build the Workspace
----------------------

Build the workspace using Colcon:

.. code-block:: bash

   cd ~/osr_ws
   colcon build --symlink-install
   source ~/osr_ws/install/setup.bash

5. Configure Realsense D435i
----------------------------

Calibrate the IMU of the Realsense D435i camera:

a. Install the pyrealsense2 library:

   .. code-block:: bash

      pip install pyrealsense2

b. Clone the Librealsense repository:

   .. code-block:: bash

      git clone https://github.com/IntelRealSense/librealsense.git
      cd librealsense/tools

c. Run the calibration script:

   .. code-block:: bash

      python3 rs-imu-calibration.py

d. Follow the on-screen instructions to position the camera in 6 different orientations.
   Aim for a calibration grade close to 98 (the achieved grade was 98.05).

6. Configure LiDAR LDS-02
-------------------------

Set up udev rules for the LiDAR:

a. Clone the TurtleBot3 repository (if not already done):

   .. code-block:: bash

      cd ~/osr_ws/src
      git clone https://github.com/ROBOTIS-GIT/turtlebot3.git

b. Copy the udev rules file:

   .. code-block:: bash

      sudo cp ~/osr_ws/src/turtlebot3/turtlebot3_bringup/99-turtlebot3-cdc.rules /etc/udev/rules.d/

c. Reload and trigger udev rules:

   .. code-block:: bash

      sudo udevadm control --reload-rules
      sudo udevadm trigger

7. Final Steps
--------------

Your SLAM/autonomous navigation project environment is now set up. Remember to source the workspace setup file in each new terminal:

.. code-block:: bash

   source ~/osr_ws/install/setup.bash
