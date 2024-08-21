# UGV-Embrapa: Monitoring of Fruit and Forestry Fields

## Overview

This project aims to develop an Unmanned Ground Vehicle (UGV) for monitoring fruit and forestry plots, inspired by NASA's Open Source Rover project. The UGV is controlled by a computer unit running ROS2 and navigates autonomously and safely between rows of agricultural fields.

> Embrapa Digital Agriculture is one of the 43 units of the Brazilian Agricultural Research Corporation (Embrapa), linked to the Ministry of Agriculture and Livestock (Mapa). Installed in Campinas (SP), it has transversal operations and a multidisciplinary team and high-performance computing infrastructure. The focus is on developing information and communication technology (ICT) solutions to meet the demands of the agricultural sector, support public policies and contribute to the open innovation ecosystem. It has partnerships with governments, research and teaching institutions and the private sector in order to encourage the use and development of emerging technologies and innovation for the sustainability of agriculture in all its dimensions. It also houses strategic initiatives such as the Science Center for Development in Digital Agriculture (CCD-AD) and the Mixed Unit for Digital Research and Innovation in Tropical Agriculture (Umipi DITAg), in addition to the Multi-User Bioinformatics Laboratory (LMB) and the Mixed Unit of Research in Genomics Applied to Climate Change (UMiP/GCCRC).

## Preparation of the Development Environment

This section describes the steps to prepare the development environment for the UGV project, including the installation of the operating system, ROS 2 Humble, and the configuration of the Raspberry Pi for control.

### Operating System Installation

1. **Download Ubuntu Server Image**:
 - Go to the [official Ubuntu Server website](https://ubuntu.com/download/server) and download the latest image.
 - Use software like Raspberry Pi Imager to save the image to the SD card.

2. **Initial Setup**:
 - Insert the SD card into the Raspberry Pi and turn on the device.
 - Configure Wi-Fi network and other basic preferences during startup.
 - Update the operating system:
 ```sh
 sudo apt update
 sudo apt upgrade
 ```

### Configuring Raspberry Pi as Server and SSH Communication

1. **Server Mode**:
 - Server configuration allows Raspberry Pi to be accessed remotely.
 - Connect to the server using SSH:
 ```sh
 ssh embrapa@10.42.0.1
 ```


### Installation Steps

1. **System Update:**
 - `sudo apt update`
 - `sudo apt upgrade`

2. **ROS 2 Humble Installation:**
 - Follow the official instructions for installing ROS 2 Humble.

### Manual Rover Bringup

To manually start the rover, run the following commands in the terminal:

1. Make sure the program is not already running:
 ```bash
 sudo systemctl stop osr_startup
 ```

2. Execute the bringup command:
 ```bash
 ros2 launch osr_bringup osr_launch.py
 ```

### Automatic Bringup with Launch Script

To automate the bringup and eliminate the need for SSH access, we configured the Raspberry Pi to launch the rover code automatically.

1. Navigate to the `init_scripts` folder:
 ```bash
 cd ~/osr_ws/src/osr-rover-code/init_scripts
 ```

2. Create symbolic links:
 ```bash
 sudo ln -s $(pwd)/launch_osr.sh /usr/local/bin/launch_osr.sh
 sudo ln -s $(pwd)/osr_paths.sh /usr/local/bin/osr_paths.sh
 ```

3. Copy the service file to the systemd services directory:
 ```bash
 sudo cp osr_startup.service /etc/systemd/system/osr_startup.service
 ```

4. Adjust service file permissions:
 ```bash
 sudo chmod 644 /etc/systemd/system/osr_startup.service
 ```

### Configuration LCD display 16x02 with I²C Conversion Module

It is necessary to supply it with 5V on one of the available power pins marked by the brain board silkscreen, as well as connect the SDA and SCL pins.

![Display Links](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/72100554/505222e8-aac1-4e31-95c6-8f10a74234f2)

After connecting it to the board, the server was accessed via SSH and the `RPi_GPIO_i2c_LCD` library was installed via the terminal:

```bash
sudo pip3 install RPi-GPIO-I2C-LCD
```

With the library installed, it was necessary to look for the address, (0x27), i2c of the display using the command:
```bash
ubuntu@embrapa:~$ i2cdetect -y 1
```
The strategy adopted to reduce consumption was to update the display within a certain time, for this it was necessary to add a command via the ‘nano’ editor to execute the script every 5 minutes:
```bash
ubuntu@embrapa:~$ crontab -e
```

In the last line the code was added:
```bash
*/5 * * * * /usr/bin/python3 /home/ubuntu/osr_ws/src/ihm_lcd.py
```

### Image Acquisition

1. **Installation of Required Libraries**:
 ```sh
 sudo apt install ros-humble-cv-bridge
 sudo apt install ros-humble-image-transport
 ```

2. **Camera Configuration**:
 - Connect Intel RealSense D435i and Logitech C920 HD PRO cameras to Raspberry Pi.
 - Use the following commands to check and adjust settings:
 ```sh
 rs-enumerate-devices
 v4l2-ctl --list-devices
 ```

3. **Publishing and Viewing Images**:
 - Run ROS nodes to capture and publish images:
 ```sh
 ros2 run realsense2_camera realsense2_camera_node
 ros2 run image_transport republish raw in:=/camera/color/image_raw out:=/image_raw
 ```
 - Another way to start capturing images is to run the code below:
 ```sh
 systemctl restart start_turtle.service
 ```
 - To view images use Rviz2 or rqt_image_view:
 - rqt_image_view:
 ```sh
 ros2 run rqt_image_view rqt_image_view
 ```
 - Rviz2:
 ```sh
 Rviz2
 ```

### ROS Humble Installation

1. **Add ROS Repository Key**:
 ```sh
 sudo apt install software-properties-common
 sudo add-apt-repository universe
 sudo apt update
 sudo apt install curl
 curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
 ```

2. **Add ROS Repository**:
 ```sh
 sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
 sudo apt update
 sudo apt install ros-humble-desktop
 ```

3. **Configure ROS Environment**:
 Add the following lines to the `~/.bashrc` file:
 ```sh
 source /opt/ros/humble/setup.bash
 ```

## Remote Control Operation

The UGV's remote control uses a Spektrum DXS radio control transmitter, which communicates with a receiver installed in the UGV. The mapping of control buttons and levers was configured to facilitate the operation of the UGV, allowing control of speed, direction, rotation and activation of specific functions. To configure the remote control, we mapped axes and buttons using the ROS 2 `joy` node.

### Controller Parameters

- **scale_linear.x**: 0.12
- **axis_linear.x**: 1
- **axis_angular.yaw**: 2
- **axis_angular.pitch**: 4
- **scale_angular.yaw**: 1.25
- **scale_angular.pitch**: 0.25
- **scale_angular_turbo.yaw**: 0.0
- **scale_linear_turbo.x**: 0.0
- **enable_button**: 0
- **enable_turbo_button**: 1

### Emergency Routine

An emergency routine was implemented to ensure the operational safety of the rover. When pressing the emergency button, the value of `scale_linear_turbo` is set to 0.0, resulting in the rover coming to a complete stop. A flowchart was created showing this routine.

_UGV operating flowchart_

![Emergency flowchart](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/49559187/f8c5dcd0-ad6a-4c44-ac0e-e2f40bb0a978)


# Mapping of Buttons and Levers

1. Speed:

 The left lever controls the speed of the UGV. By moving the lever forward, the UGV moves forward; moving it backwards, the UGV retreats. This setup is intuitive
 and allows precise speed control.

2. Direction:

 The right lever is responsible for steering the UGV. By moving the lever to the left or right, the UGV rotates in the respective direction. This control is
 essential for navigation in varied terrain.

3. On/Off:

 The lower central button allows you to turn the UGV on or off. This function is crucial to start or stop the UGV operation quickly.

4. Emergency:

 The emergency button, located at the top left of the control, can be activated to immediately stop the UGV. This button is mapped to adjust the value of scale_linear_turbo to 0.0, resulting in the vehicle coming to a complete stop in emergency situations.

5. Rotation:

 The rotation of the UGV is controlled by a switch at the top right of the controller. This functionality is used for fine adjustments of rotation,
 allowing precise maneuvers.

6. Deadman:

 The Deadman button, located on the back of the controller, must be pressed continuously to keep the UGV operating. Releasing this button results in stopping
 immediate release of the UGV, as an additional security measure.

# Operation
To operate the UGV, the user must:

1. Connect the Control and UGV:

 Press the power button on the controller and then turn on the UGV.

2. Adjust Speed ​​and Direction:

 Use the levers to control the speed and direction of the UGV as needed for the mission.

3. Monitor the Operation:

 Be aware of the UGV's response and adjust controls as necessary. Use the rotation lever and emergency button for quick maneuvers and stops.

4. Security:

 Always keep one finger on the Deadman button to ensure safe operation. In case of emergency, use the emergency button to stop the UGV immediately.

 # Controller photo

![Controller front](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/49559187/aa9ca24c-7300-449e-a2a1-b97dc7d0b22e)

![Controller behind](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/49559187/849eaf65-5603-4b8a-b43f-8160a4e89344)

## Results

### Visualization and Monitoring

1. **Graphic Interface (Rviz2)**:
 - Used to view ROS topics, including images captured by cameras.
 - Allows you to monitor the position and status of sensors in real time.

2. **Image Acquisition**:
 - Intel RealSense D435i cameras capture images with depth, allowing the creation of 3D maps of the environment.
 - Images are published in ROS topics and can be viewed remotely.

### Performance and Processing

1. **Real-Time Processing**:
 - The server mode configuration of the Raspberry Pi, combined with ROS, allows efficient processing of captured images.
 - The system was able to handle processing multiple image streams simultaneously without significant delay.

2. **Stability and Reliability**:
 - The system demonstrated stability during testing, with SSH communication allowing safe and effective remote control.
 - Integration with ROS made it possible to easily add and modify nodes as needed, ensuring flexibility for future expansions.

## Contributors

- Breno Alencar Araújo
- Bruno Morales Balkins
- Fernando Bichuette Assumpção
- Giulia Carolina Martins de Sampaio
- Felipe Catapano Emrich Melo
- Rafael Eli Katri
- Gabriel Tumang
- Luana de Matos Sorpreso

Advisor: Prof. Dr. Vinícius Licks

Mentor: Thiago Teixeira Santos
