# Bicopter
Bicopter is unmanned aerial vehicle (UAV) with only two thrust motors and two steering motors that control the rotation of the thrust motors, changing the thrust motors' force direction and compensating for the lower number of thrust motors compared to multi-rotor UAVs (like quadcopter). This tilting mechanism allows the bicopter to balance and move in space, as well as rotate the thrust motors so that they are oriented horizontally, as with the Bell Boeing V-22 Osprey.

![alt text](https://github.com/Ihab-Asaad/Bicopter/blob/master/images/bicopter.PNG)

# Bicopter design:
To visualize the behavior of a real bicopter, the bicopter parts has been designed and assembled Autodesk Inventor. The body frame of the bicopter was imported from RCExplorer (https://rcexplorer.se/product/bicopter-electronics-kit/)  provided as Inventor files. Other parts like propellers, steering motors (servo), battery, thrust motors (brushless dc motors), have been designed to match the prototype of RCExplorer kit. The body coordinate frame was positioned in the center of the bicopter mass in the Inventor prototype. Finally the bicopter has been exported to Matlab-SimMechancis.

# Matlab Simulation:
Bicopter has 6 degrees of freedom but only 4 input control  ({F_1},{F_2},{α_1},{α_2}).  It  is  an  under  actuated  system.
The inputs have been added to the exported model. {F_1},{F_2} have been saturated to match the real forces that the thrust motors can provide. α_1,α_2 too, as the design doesn’t allow steering motors to perform a full rotation.
The drag force f has been appended to the center of bicopter.
