Mechanical Assembly
===================

Overview
--------

The robot's structure is an adaptation of unmanned vehicles that monitor the surface of Mars and the moon. It features a Rocker-Bogie suspension made of aluminum profile, developed in 1996 for NASA's Sojourner rover. This suspension system keeps the chassis level even on uneven terrain, minimizing rolling and pitching.

The suspension consists of two axles:

1. The first axle is fixed to the chassis and rotates the larger arm (rocker).
2. The second axle is at the end of the first and rotates the smaller arm (bogie).

Wires from the motors and encoders pass through these axles to the inside of the chassis.

Chassis Design
--------------

The robot's chassis is composed of:

- Aluminum bars
- Acrylic sheets

These components house the device's electronic boards, batteries, and HMI. Camera and LiDAR supports are attached to the chassis.

Design Considerations:

- The robot's center of mass is positioned above the unsprung mass (wheels and suspension) for greater stability.
- Maximum incline is limited to angles less than 45° due to the height of the center of mass.
- Obstacles exceeding the combined torque of the six motors or with a height greater than the wheel radius cannot be overcome.

Assembly Instructions
---------------------

**For detailed instructions on base assembly, including the main body frame and rocker-bogie suspension, refer to the Open Source Rover official docs:** https://github.com/nasa-jpl/open-source-rover/tree/master/mechanical.

.. figure:: _static/base_structure.png
   :alt: Base structure
   :width: 50%
   :align: center

   Base structure.

- Wheel assembly subsystem: https://github.com/nasa-jpl/open-source-rover/blob/master/mechanical/wheel_assembly/README.md

- Main body subsystem: https://github.com/nasa-jpl/open-source-rover/blob/master/mechanical/body/README.md

- Rocker-Bogie suspension subsystem: https://github.com/nasa-jpl/open-source-rover/blob/master/mechanical/rocker_bogie/README.md

Component Adaptations
---------------------

Some parts may need adaptation based on component availability:

Aluminum Profiles
^^^^^^^^^^^^^^^^^

If 96 mm profiles are unavailable:

1. Purchase larger profiles and cut to the correct dimension.
2. Redo threading for M4 screws in all four holes of each cut part.
3. Use taps of different sizes in three stages to achieve the desired thread size.

.. figure:: _static/aluminum_profile.png
   :alt: Aluminum profile
   :width: 50%
   :align: center

   Cut aluminum profile.

.. figure:: _static/threading.png
   :alt: Threading
   :width: 50%
   :align: center

   Threading operation.

Drilling Operations
^^^^^^^^^^^^^^^^^^^

After cutting the parts, it may be necessary to drill the bars, since the existing hole may not be through like in the profile. In addition, it may also be necessary to perform threading in two stages, as was the case with the other part mentioned above.

If drilling is necessary, follow these steps:

1. Paint the face to be drilled with a marker.
2. Mark the hole center using a vertical height marker.
3. Create a central mark using a center punch, vice, and hammer.
4. Drill using a manual benchtop milling machine, ensuring proper alignment and chip evacuation.

.. figure:: _static/drilling_marking.png
   :alt: Drilling marking
   :width: 50%
   :align: center

   Marking for drilling operation.

Bearing Substitution
^^^^^^^^^^^^^^^^^^^^

If the original bearing part is unavailable:

- Use a substitute with an equivalent internal diameter.
- Note that slight structural deformation may occur but should not compromise operation.

.. figure:: _static/substitute_bearing.png
   :alt: Substitute bearing
   :width: 50%
   :align: center

   Substitute bearing.

Wheel Fixation
^^^^^^^^^^^^^^^

Due to the torque of the engines, it may be necessary to modify the wheel attachment.

Originally, the attachment is established between the engine shaft and the wheel with a central screw pressing the wheel between a washer and the wheel hub attached to the engine shaft. After the new attachment, the wheels are drilled and attached, with two screws each, to the wheel hubs. However, it was found that this strategy, suggested in the official project repository, is prone to misalignment.

.. figure:: _static/wheel_fixation.png
   :alt: Wheel fixation
   :width: 50%
   :align: center

   Example of attaching the wheel to the hub


.. note::
   As of the writing of this document, there is an error in the official documentation:

   - Component initially coded as 1601 actually corresponds to component number 1611.
   - The 3D assembly view shows a part with an internal diameter less than 8 mm, which does not fit the specified axle.

Autonomous Navigation/SLAM Module
---------------------------------

To allow the robot to navigate autonomously, perform SLAM and communicate with the robot's HMI, it is necessary to make the casing that houses the navigation module and supports for the robot's sensors, then mount the navigation module and sensors on the case (on top of the base platform structure).

Acrylic Case
^^^^^^^^^^^^^

Cutting files for the acrylic case, separated into walls, bottom and top, are available in the "Mechanics/Modulo autonomo" folder. The .dxf files can be opened with tools such as RDWorks or Rhino, and .prt files can be opened with tools such as Siemens NX.

It is necessary to use a laser cutting machine to cut the acrylic sheets, then screw the bottom part to the base structure. The top part simply slides in place, above the case walls.

.. figure:: _static/optiplex_case.jpg
   :alt: Acrylic case
   :width: 33%
   :align: center

   Acrylic case.

.. figure:: _static/optiplex_L_screws.jpg
   :alt: L-screws
   :width: 33%
   :align: center

   L-screws used to fix the bottom part to the base structure.

Cooler Fan Hole
^^^^^^^^^^^^^^^^

A hole is required to intall the cooler fan in front of the step-up converter module, which is used to power the Optiplex board for the navigation module.
To achieve this, a 80mm hole was drilled in the front wall of the chassis, and the fan was fixed via 4x M3 screws and nuts.
