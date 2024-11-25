Launch Package
===============

The osr_autonomous package contains the launch files for the robot, for operation using the SLAM and navigation stack described in previous sections (for the Raspberry Pi's launch package, refer to the Raspberry Pi's environment setup section).

Usage
-----

1) First, make sure the package is built and sourced in your ROS 2 workspace:

   .. code-block:: bash

      cd UGV-Embrapa/Software/osr_autonomous
      colcon build --packages-select osr_autonomous
      source install/setup.bash

   .. note::
      You can also source the ROS 2 environment from the Environment Setup - Dell Optiplex section. Refer to it for more information.

2) Then you can launch any of the files using the ros2 launch command with the package name and launch file name:

   .. code-block:: bash

      ros2 launch osr_autonomous gps_launch.py

      ros2 launch osr_autonomous ir_depth_launch.py

      ros2 launch osr_autonomous rosbag_launch.py

      ros2 launch osr_autonomous rtab_imu_launch.py

      ros2 launch osr_autonomous stereo_launch.py

3) For the rosbag_launch.py, you can specify a custom directory for saving the bag files:

   .. code-block:: bash

      ros2 launch osr_autonomous rosbag_launch.py bag_dir:=/path/to/save/bags

   .. note::

      Also, ensure that:

      - Your RealSense D435i camera is properly connected
      - If using GPS features, your GPS device is connected and configured
      - The robot's URDF is properly set up in the osr_gazebo package
      - Parameter files exist in the correct locations:
         - ``osr_autonomous/realsense_params/params.yaml``
         - ``osr_autonomous/nav2_params/params.yaml``
         - ``osr_autonomous/nav2_params/map.yaml``

Configuration Notes
--------------------

   - All launch files support parameter overrides via command line arguments
   - RTAB-Map parameters are optimized for real-time performance
   - Transform configurations assume standard sensor mounting positions
   - Nav2 parameters and maps are loaded from the osr_autonomous package

Launch Files
------------

The launch files are designed to support different operational modes and sensor configurations for the UGV. Each launch file serves a specific purpose in the robot's operation, from basic visual SLAM to full GPS-aided navigation.

gps_launch.py
^^^^^^^^^^^^^

This launch file implements a complete navigation stack with GPS integration. It's ideal for outdoor operations where GPS signals are available.

Key Features:

   - Initializes RealSense D435i camera in IR/depth mode
   - Enables IMU (gyroscope and accelerometer)
   - Sets up RTAB-Map for visual SLAM
   - Configures Nav2 for autonomous navigation
   - Integrates GPS data via the NMEA driver
   - Uses robot_localization for sensor fusion
   - Includes GPS waypoint following capability

ir_depth_launch.py
^^^^^^^^^^^^^^^^^^

A simplified version focused on indoor navigation using IR and depth data from the RealSense camera.

Key Features:

   - Configures RealSense D435i for IR and depth streaming
   - Sets up RTAB-Map for visual SLAM
   - Enables Nav2 for autonomous navigation
   - Uses IMU data for improved odometry
   - Excludes GPS-related components

rosbag_launch.py
^^^^^^^^^^^^^^^^

Similar to gps_launch.py but includes ROS bag recording functionality for data collection and post-processing.

Key Features:

   - Includes all functionality from gps_launch.py
   - Adds automatic ROS bag recording of all topics
   - Configurable output directory for recorded bags
   - Useful for development and debugging

lidar_launch.py
^^^^^^^^^^^^^^^

Similar to gps_launch.py but with LiDAR integration for improved mapping and navigation (complete stack without bag recording).

Key Features:

   - Includes all functionality from gps_launch.py
   - Adds LiDAR data integration
   - Improved mapping and navigation capabilities
   - No ROS bag recording

rtab_imu_launch.py
^^^^^^^^^^^^^^^^^^

Specialized configuration focusing on RGB-D SLAM with heavy IMU integration.

Key Features:

   - Uses RGB color stream instead of IR
   - Enhanced IMU integration with RTAB-Map
   - Configured for improved 3D mapping
   - Forces 2D constraints for better ground robot operation
   - More aggressive feature detection settings

stereo_launch.py
^^^^^^^^^^^^^^^^^

Basic stereo vision configuration using the RealSense camera's IR sensors.

Key Features:

   - Minimal configuration using IR and depth
   - No IMU integration
   - Basic RTAB-Map configuration
   - Lighter weight than other launches
   - No navigation stack integration