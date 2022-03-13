<!-- # Bicopter
Bicopter is unmanned aerial vehicle (UAV) with only two thrust motors and two steering motors that control the rotation of the thrust motors, changing the thrust motors' force direction and compensating for the lower number of thrust motors compared to multi-rotor UAVs (like quadcopter). This tilting mechanism allows the bicopter to balance and move in space, as well as rotate the thrust motors so that they are oriented horizontally, as with the Bell Boeing V-22 Osprey.

![alt text](https://github.com/Ihab-Asaad/Bicopter/blob/master/images/bicopter.PNG)

## Bicopter design:
To visualize the behavior of a real bicopter, the bicopter parts has been designed and assembled Autodesk Inventor. The body frame of the bicopter was imported from RCExplorer (https://rcexplorer.se/product/bicopter-electronics-kit/)  provided as Inventor files. Other parts like propellers, steering motors (servo), battery, thrust motors (brushless dc motors), have been designed to match the prototype of RCExplorer kit. The body coordinate frame was positioned in the center of the bicopter mass in the Inventor prototype. Finally the bicopter has been exported to Matlab-SimMechancis.

## Matlab Simulation:
Bicopter has 6 degrees of freedom but only 4 input control  ![image_](https://latex.codecogs.com/gif.latex?F_1%2CF_2%2C%5Calpha_1%2C%5Calpha_2).  It  is  an  under  actuated  system.
The inputs have been added to the exported model. {F_1},{F_2} have been saturated to match the real forces that the thrust motors can provide. α_1,α_2 too, as the design doesn’t allow steering motors to perform a full rotation.
The drag force f has been appended to the center of bicopter. -->

# Bi-copter Simulation and Control:
This project demonstrates a Bi-copter desinged in AutoDesk Inventor and controlled in Matlab Simulink environment.

# Contents:
Requirements.
Quick Start.
Controllers.
Euler Angles.
Noise & Filters. 
Attitude and altitude control/Position Control.



## Requirements:
Matlab 2019b or higher versions.

## Quick Start:
Download the project to your local. Open Matlab and navigate to your local folder containing the project. Finally, Run Sim_Script.m file.

## Controllers:
The following controllers have been applied to control Bi-copter's attitude and altitude:
- PID.
- Super Twisting Sliding Mode Controller(STSMC).
- Linear/Nonlinear active disturbance rejection control (ADRC).
- ADRC with Kalman.
- Fuzzy logic control (FLC)

you can choose any of them by changing 'method' parameter in Sim_Script.m file.

## Euler Angles:
you can choose between euler angles getten from 6-DOF joint or from integrating angular speeds of Bi-copter using 'euler_source' parameter.

## Noise & Filters:
low-pass filters are used to:
- Filter noise added to angular speeds.
- Filter the output of D-term in the chosen controller.
- Simualate the behave of motors in real Bi-copter (four low-pass filters where added the four inputs).

you can change the parameter of each filter in Sim_Script.m file.

## Attitude and altitude control/Position Control:
Choose between controlling Bi-copter attitude and altitude or position control in space using 'method_angle_plane_sw' parameter.
