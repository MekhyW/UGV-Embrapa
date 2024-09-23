# Mechanics

## 3D Model
The base 3D model can be found in [OnShape](https://cad.onshape.com/documents/e4f00b1a3d2edb1a84bbba1c/w/8ab8f394324bcc586236ef5d/e/9191e5ad2a70f387b419bc55?renderMode=0&uiState=645ede92f3a1a9205158b296) or in the STEP file provided above.

![image](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/72100554/a2137652-97fa-4312-889d-916f1c728e59)


## Adaptations to Official Documentation
Some adaptations were necessary in the assembly of the official documentation, since the components were imported and some of them were not available for delivery. To get around this problem, equivalent parts were purchased that would allow adaptation for the proposed application.
The parts that underwent this adaptation process are listed and commented below.
### Suspension Profiles
The first parts to be adapted were the aluminum profiles that make up part of the robot's suspension, since it was not possible to find them in the lengths specified in NASA's official documentation, 96 mm. As an alternative, larger profiles were purchased and cut to the correct size.

![image](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/62897902/4b00c4d5-f164-4cf2-847e-b903f16ecaa5)

After cutting the profiles, the thread for the M4 screw used in the assembly had to be redone for all four holes in each of the cut parts. The threading operation was carried out manually and consisted of three stages, using taps of different sizes until the desired thread size was reached. The part did not need to be drilled again, since its holes were through holes, covering the entire structure, regardless of length.

![image](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/62897902/6dc17718-f09a-4de4-a52c-d962fd01e204)

### Aluminum Body Bars

The aluminum bars used to assemble the robot body also needed to be adapted, since they were also purchased in a length greater than that specified and then cut to the correct size, 96 mm.

After performing the cutting operations on the parts, it was necessary to drill the bars, since the existing hole was not through like in the profile. In addition, it was also necessary to perform the threading in two stages, as was the case with the other part mentioned above.

During the drilling operation, the following steps were performed to ensure better alignment of the hole:

1. Painting the face to be drilled with a marker.

2. Removing the paint to mark the hole with the aid of a vertical height marker. The line was drawn with a dimension equivalent to half the side of the face, ensuring that the straight lines intersect in the center.

3. Making a central mark to ensure that the drill is not inserted misaligned, a step performed with a center punch, a vice and a hammer.

4. Drilling with the aid of a manual benchtop milling machine, aiming to align the tip of the drill with the marking made in the previous step. Advance and retreat the drill during the process, ensuring chip evacuation and dissipation of the heat generated.

![image](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/62897902/f95e6519-0d76-4535-a70d-1e3676e8d23a)

The threading process was also necessary and was redone in a manner identical to the procedure described for the profiles.

The part with the bearing for fitting the suspension was also not available in the original model in NASA documentation, so another with an equivalent internal diameter was used (Figure 28). Although the replacement did not fit perfectly and caused slight structural deformation, it was decided not to make any adaptations because the components could be damaged and there were no substitutes. The resulting deformation effect did not compromise the rover's operation.

![image](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/62897902/749860f7-e1b6-4cc8-a596-2a6e98edfee1)

## Review of Official Documentation Errors

One of the suspension components was incorrectly identified in the official documentation, with initial code 1601, which in reality corresponds to component number 1611. In the 3D view of the assembly, the part identified with the code corresponding to that informed in the official documentation has an internal diameter with a value of less than 8 mm, not assembling on the specified axle.

![image](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/62897902/36c3ddc6-d7ef-491b-82d5-7051012ea459)

## Additional Components Manufactured

To ensure the robot's adaptability to the monitoring environment, some additional components needed to be manufactured, including the camera and LiDAR supports, allowing these components to be placed in the structure in the future.

### Camera Support

As requested by the client, some supports for cameras and a LIDAR were also developed, using Fusion 360 software for modeling and 3D printing technology to materialize the project.

<img width="470" alt="suporte1" src="https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/38721933/3ee1b43e-3dae-476a-853d-57575b2d889a">

<img width="474" alt="Screenshot at Jun 10 11-16-50" src="https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/38721933/ca6b2074-cb1d-4ac4-968d-367030aa3d81">


### LiDAR Support

<img width="460" alt="suporte3" src="https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/38721933/a7b702c6-86ae-4b73-a62c-17ce572221a8">

