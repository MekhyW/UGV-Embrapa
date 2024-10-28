Schematics and Wiring
=====================

Prerequisites
--------------
The prototype's electronics are made up of two boards: the motor drive board (Motor Board), where the power circuits are located, and the board responsible for controlling the entire system's operation (Brain Board), where the control circuits are located and a Raspberry Pi 4, model B, 8 GB, with a 64-bit quad-core ARM-Cortex A72 microprocessor, running at 1.5 GHz, is located.

The main electronics are based on the original platform schematics found at https://github.com/nasa-jpl/open-source-rover/blob/master/electrical/pcb/control_board/documentation/v2.0.1/schematics.pdf, and the following diagrams:

.. figure:: _static/electronic_boards.png
   :alt: Electronic boards
   :width: 80%
   :align: center

   Electronic boards.

.. figure:: _static/electrical_diagram.png
   :alt: Electrical diagram
   :width: 80%
   :align: center

   Electrical diagram.

.. figure:: _static/signal_diagram.png
   :alt: Signal diagram
   :width: 80%
   :align: center

   Signal diagram.

Power Supply Setup
----------------------
a) Battery Configuration
   
   - Use a 14.8V DC battery
   
   - Connect through protection circuit containing:
        
        * Fuse
        * 1 kΩ resistor
        * Protection diode

b) Voltage Regulation System
   
   - Install voltage regulators:
    
        * 14.8V → 12V (D24V22Fx) for Roboclaw drivers
        * 14.8V → 5V (D24V150Fx) for main systems
        * 5V → 3.3V (MIC2937A-3.3WT) for low-power components

c) Power Distribution

   12V powers:

   - Roboclaw motor drivers

   5V powers:
   
   - Raspberry Pi
   
   - PWM expansion module
   
   - Servo motors
   
   - HMI display
   
   - Motor encoders

   3.3V powers:
   
   - PWM expansion (PCA9685)
   
   - INA260 monitor
   
   - Status LEDs

Motor System Installation
----------------------------

There are two different types of motors being used: brushed DC motors, used to propel the system, and servos, used to change the robot's orientation.

The DC motors are the “5203 Series Yellow Jacket Planetary Gear Motor” model, manufactured by GoBilda, and have a maximum torque of 38 kg.cm. A total of 6 motors are used to move the robot, each equipped with a magnetic encoder, which performs the reading through a hall effect sensor. These readings can be monitored in a specific node, ‘roboclaw_movemotor.py’, contained in: osr-rover-code/scripts/ in the repository of only the robot codes.

The servo motors used are the “Dsservo DS3225” model and have a maximum torque of 21 kg.cm with the 5 V supplied by the drive board. In total, 4 actuators of this type are required to control the robot's orientation, enabling lateral dynamics.

a) DC Motors
   
   - Install 6x GoBilda 5203 Series Yellow Jacket Planetary Gear Motors
   
   - Specifications: 38 kg.cm maximum torque
   
   - Connect magnetic encoders to each motor
   
   - Wire to Roboclaw drivers

.. figure:: _static/dc_motor.png
   :alt: DC motor
   :width: 50%
   :align: center

   DC motor.

b) Servo Motors
   
   - Install 4x Dsservo DS3225
   
   - Specifications: 21 kg.cm at 5V
   
   - Connect to PCA9685 PWM expansion module

.. figure:: _static/servo_motor.png
   :alt: Servo motor
   :width: 50%
   :align: center

   Servo motor.

Control Systems Setup
----------------------
Considering the communication between the microcomputer and the voltage and current monitoring module (INA260), it is noted that data is also sent via an I2C communication protocol. The module also communicates with the signaling LEDs to alert the user if its internal shunt resistor detects any change in the power supply system.

a) PWM Control
   
   - Install PCA9685 module
   
   - Connect to Raspberry Pi via I2C
   
   - Wire 16 PWM channels as needed

In addition to the expansion module and the monitor, communication with the HMI display, which will be implemented in the future, will also take place using the I2C protocol. The reason for this choice is that the control board has an available I2C bus, which could be adapted for connecting the display with a low number of adaptations required.

Another important observation regarding the diagram is that the Raspberry Pi is also responsible for controlling the 12V voltage regulator through an enable signal, turning the 12V output on or off.

Analyzing the control board a little more, we can see the presence of some signaling LEDs. These components are intended to communicate with the user, signaling, for example, when data is being transferred via the serial port, lighting up when the bits are transferred.

b) Monitoring System
   
   - Install INA260 module
   
   - Connect via I2C to Raspberry Pi
   
   - Wire status LEDs for power monitoring

Safety Systems
---------------
To ensure the prototype's compliance with the occupational safety standard for machines and equipment (NR-12), it was necessary to include an emergency circuit, containing a physical emergency button and ensuring the robot's complete stop, in addition to de-energizing the control and power boards, through a normally closed switch, responsible for cutting off the power supplied to both boards.

a) Emergency Circuit
   
   - Install physical emergency button (NC switch)
   
   - Wire to power cutoff system for both boards
   
   - Configure remote emergency via radio control

.. figure:: _static/emergency_circuit.png
   :alt: Emergency circuit
   :width: 80%
   :align: center

   Emergency circuit.

b) Status Indicators
   - Install status LEDs for:
        
        * Serial communication
        
        * Power status
        
        * Emergency status

Display Installation
---------------------
To display relevant information about the robot's status, such as connectivity, battery percentage, etc., a 16x02 LCD display with an I²C conversion module was installed, as provided in the project's solution matrix. It was necessary to supply it with 5V on one of the available power pins indicated by the brain board's silkscreen, as well as connect the SDA and SCL pins.

- Install 16x02 LCD display with I²C module

- Connect to 5V power

- Wire SDA and SCL pins to Raspberry Pi

.. figure:: _static/display_wiring.png
   :alt: Display wiring
   :width: 80%
   :align: center

   Display wiring.

Wiring Guidelines
------------------
The wiring required for power supply, encoder reading and signal transmission to the actuators followed the instructions in the GitHub repository at: https://github.com/nasa-jpl/open-source-rover/tree/master/electrical/wiring. The suggested dimensions, however, were reduced, so an excess of at least 10 cm was added to each wire. The color code was respected as well as the gauge of the motor power wires. Wires for control signals were replaced with smaller AWG gauge two-piece.

- Add 10cm extra length to all specified wire lengths

- Follow color coding scheme

- Use specified AWG for motor power

- Use smaller AWG for control signals

Testing and Verification
------------------------
- Test voltage levels at all regulation points

- Verify emergency shutdown functionality

- Test motor control systems

- Confirm encoder readings using roboclaw_movemotor.py

- Verify I2C communication with all peripherals

Note: Always refer to the original schematics for detailed connection diagrams and component specifications.
