Overview
========

The objective of the project is to implement an autonomous navigation and trajectory monitoring system for a UGV, that is, an unmanned ground vehicle. The vehicle must be able to plan its movement and send positioning information while navigating through field corridors, the basic unit of agricultural planting. The second iteration is part of a larger context of UGV projects for agricultural use, representing continuity with the first one, responsible for assembling the robot to be used, capable of operating in orchards and forestry fields.

.. raw:: html
   :file: _static/presentation.html

About Embrapa Digital Agriculture
---------------------------------

The Brazilian Agricultural Research Corporation (Embrapa) is a federal company founded in 1973 and linked to the Ministry of Agriculture and Livestock (MAPA), focused on technological development and research applied to Brazilian agriculture, in a manner compatible with the nuances of the tropical climate and the socioeconomic particularities of the various regions of a continental country.

Why is this project important?
------------------------------

In agriculture, whether fruit growing or forestry, a fundamental element is the plot.

The plot is the minimum cultivation unit on a property, generally delimited by natural characteristics such as relief, soil type, or by artificial characteristics such as roads and fences.

In fruit growing, plots are often used to organize the planting of fruit trees. Each plot can contain a single variety of fruit, allowing for more efficient tree management and a more organized harvest. In addition, plots allow for more effective control of pests and diseases, as it is easier to isolate an affected plot for treatment.

In forestry, plots are even more important. They are used to organize the forest into management units, each with its own forest management plan. This allows different silvicultural techniques to be applied in different parts of the forest, depending on the specific characteristics of each plot. In addition, plots make it easier to conduct forest inventories and plan harvesting operations.

In the fields, monitoring during the production process is essential, enabling production and harvest planning, as well as pest control, ensuring greater quality of the product offered to the end consumer and reducing losses on the part of producers.

The human monitoring process can be difficult or even impossible depending on factors such as the size and number of plots. According to data from the Brazilian Confederation of Agriculture and Livestock (CNA), in 2021, national fruit production exceeded 41 million tons, with 81% of producing establishments being classified as family farming. According to the IBGE (Brazilian Institute of Geography and Statistics), Brazil, which is the leader in the supply of wood for the production of paper and pulp, broke a record in 2022, with 99.7 million cubic meters. The states of the Southeast lead both productions, however, only 16% of agricultural establishments are family farming in Brazil. Data that reveal the strength of the primary sector in the Brazilian economy and that large estates, concentrated in this region, have been demanding new technologies to improve productivity.

Considering such demand, the development of technologies in the sector applied to the monitoring of plantations can ensure that the procedure is carried out in a more assertive and rapid manner.

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

