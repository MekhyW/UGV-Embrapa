# Electronics 

## Schematics and General Operation Details 

The prototype's electronics are composed of two boards: the motor drive board (Motor Board), where the power circuits are located, and the board responsible for controlling the entire system's operation (Brain Board), where the control circuits are located and a Raspberry Pi 4, model B, 8 GB, with a 64-bit quad-core ARM-Cortex A72 microprocessor, running at 1.5 GHz, is located.

![image](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/62897902/1dc84aa9-ae4c-47ea-b9fe-22ce198eccc0)

### Power Supply Diagrams

First, analyzing the system power supply, a battery supplying 14.8V DC is connected directly to the drive board, passing through a protection circuit, which has a fuse, a 1 kΩ resistor and a diode, which act as protection for the motor circuit. In addition, a voltage and current monitoring module (INA260) is used to measure overcurrent, in addition to a multimeter, which is used for user monitoring of current and voltage data.

![image](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/62897902/9f748100-b6f7-4ea9-af1d-d4a25ad7c739)


The 14.8V voltage is used to power two regulators, a 12V step-down regulator (D24V22Fx), used to power the Roboclaw drivers, and a 5V regulator (D24V150Fx). There is also a third voltage regulator that converts the voltage from 5V to 3.3V (MIC2937A-3.3WT), powering the PWM expansion module (PCA9685), the voltage and current monitor, and the signaling LEDs on both boards. The 5V voltage, in turn, also powers the PWM expansion module, the servos, the Raspberry Pi, the HMI display, and the motor encoders. The motors are powered by Roboclaw via PWM with a maximum voltage of 12V.

### Signal Diagrams

For the system signal diagram, note that a new block has been added, the set between the operator and the radio control, responsible for sending commands via radio waves to the Raspberry Pi, which receives the signal via a USB module. The data received by the microprocessor is sent via serial and used to control the motors and servos.

![image](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/62897902/25b216b0-da69-4218-9a60-26376cbdb3fe)

The control loop for the six motors is controlled by the Raspberry Pi, which receives the reference from the radio control and sends the commands to the Roboclaw drivers, which perform the activation via PWM. The loop is closed by the encoders, which are responsible for the sensing, sending the collected data to the controller present in the driver.

The four servos, in turn, are controlled in an open loop (internally closed) activated by means of a PWM signal. This signal is provided by the PCA9685 module (expansion), used to increase the number of servos that can be controlled, by generating 16 new PWM channels. Communication between this module and the Raspberry Pi occurs via the I2C communication protocol.

Considering the communication between the microcomputer and the voltage and current monitoring module (INA260), it is noted that data is also sent via an I2C communication protocol. The module also communicates with the signaling LEDs to alert the user if its internal shunt resistor detects any change in the power supply system.

In addition to the expansion module and the monitor, communication with the HMI display, which will be implemented in the future, will also occur according to the I2C protocol. The reason for this choice is that the control board has an available I2C bus, which could be adapted for connecting the display with a low number of necessary adaptations.

Another important observation regarding the diagram is that the Raspberry Pi is also responsible for controlling the 12V voltage regulator through an enable signal, turning the 12V output on or off.

A closer look at the control board reveals the presence of some signaling LEDs. These components are intended for communication with the user, signaling, for example, when data is being transferred via the serial port, lighting up when the bits are transferred.

Finally, the last data stream to be analyzed is the emergency status indicator, in which a high logic level digital signal is sent by the emergency button to the microprocessor, which communicates the situation to the user via the HMI display.

## Physical Emergency Circuit

The schematic of the physical emergency circuit used to de-energize the robot is illustrated below.

![image](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/62897902/a42c251f-cbff-423a-822a-de9d12807db7)

To indicate the emergency status to the user, a red indicator light was adopted, which lights up as soon as the button is pressed. The component has an internal resistance of 1800 Ω, supporting voltages of up to 24V and is activated by a normally open switch, responsible for closing the circuit after the button is pressed.

A remote emergency button was also implemented, whose operation is explained in the programming folder.

## Display Circuit Details

The schematic of the display pin connection to ensure I2C communication is shown below. The pins for connecting 5V and GND are available on the Raspberry Pi and an extension of the SCL and SDA pins can be found on the control board.

![image](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/62897902/89fe5d82-cf03-44a6-9ce3-b9f463eb3dc9)
