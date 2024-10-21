Overview
========

The UGV-Embrapa project aims to develop an Unmanned Ground Vehicle (UGV) for monitoring fruit and forestry plots, inspired by NASA's Open Source Rover project. The UGV is controlled by a computer unit running ROS2 and navigates autonomously and safely between rows of agricultural fields.

.. raw:: html
   :file: _static/presentation.html

About Embrapa Digital Agriculture
---------------------------------

Embrapa Digital Agriculture is one of the 43 units of the Brazilian Agricultural Research Corporation (Embrapa), linked to the Ministry of Agriculture and Livestock (Mapa). Located in Campinas (SP), it focuses on developing information and communication technology (ICT) solutions for the agricultural sector, supporting public policies, and contributing to the open innovation ecosystem.

Key Features
------------

- Autonomous navigation in agricultural fields
- ROS2 Humble-based control system
- Remote control operation using Spektrum DXS radio transmitter
- Image acquisition using Intel RealSense D435i and Logitech C920 HD PRO cameras
- Real-time processing and monitoring capabilities

Development Environment
-----------------------

The project's development environment includes:

- Ubuntu Server as the operating system
- ROS 2 Humble for robotics framework
- Raspberry Pi configured as a server for remote access and control

Key Components
--------------

1. **Remote Control**: Utilizes a Spektrum DXS radio control transmitter for manual operation.
2. **Image Acquisition**: Employs Intel RealSense D435i and Logitech C920 HD PRO cameras for capturing visual data.
3. **Visualization**: Uses Rviz2 for real-time monitoring and visualization of sensor data.
4. **Automatic Bringup**: Implements a launch script for automated startup of the rover system.

Safety Features
---------------

The UGV includes an emergency routine for operational safety:

- Emergency button on the remote control
- Deadman switch for continuous operation
- Automatic stop functionality when control signals are lost

Results and Performance
-----------------------

- Successful real-time processing of multiple image streams
- Stable and reliable operation during testing
- Flexible system architecture allowing easy addition and modification of ROS nodes

Future Directions
-----------------

The UGV-Embrapa project provides a foundation for further research and development in agricultural robotics, with potential for expanding capabilities in autonomous navigation, crop monitoring, and precision agriculture.

