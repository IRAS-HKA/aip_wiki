# Repository Overview

This section provides a general overview of the used repositories for the AIP application. 


## 1. AIP-Coordinator

This docker container handels the control via the Groot Behavior Tree System.
You can design new behavior trees and programm the sequence visually.
Alternativly, you can change existing behavior trees.

The full instructions and explaination of the code can be found here:
[GitHub Repository](https://github.com/IRAS-HKA/aip_coordinator)


## 2. Aip_Cell_Description

This repository contains the specific describtion of the robot in this case KUKA KR10. 
[GitHub Repository](https://github.com/IRAS-HKA/aip_cell_description/tree/main)

Without this repo the simulation has no representation of the robot. The repo handels the kinematics, urdf files (= description of the robots dimensions, used angels) and how to handle collisions.

If those information would be missing, the robot could crash with the environment, simply because the environment is not known.

## 3. AIP_BringUp 
   
The full instructions and explaination of the code can be found here:
[GitHub Repository](https://github.com/IRAS-HKA/aip_bringup)

This repo explains in detail who to connect to the robot, however this contains the information of the ready2educate cells.
The repository can still be used, we just need to add the KUKA KR 10 parameters.
The explaination how to control the KUKA SmartPad in order to connect with ROS 2 is done in How_To_Start_AIP.

After cloning the repository, you can build and start the docker container.
This enables you to move the robot after initilising the robot driver. The example movement is defined in test_ros_env.

For the further details please take a look in the GitHub repository.


## 4. Kuka_Eki 

The full instructions and explaination of the code can be found here:
[GitHub Repository](https://github.com/IRAS-HKA/kuka_eki/tree/driver-humble)

This repo contains the configurations of the KUKA EKI Interface, which is used to build up an connection between the linux-pc and the robot control-unit.


### 5. MoveIt Wrapper

The full instructions and explaination of the code can be found here:
[GitHub Repository](https://github.com/IRAS-HKA/moveit_wrapper)

Custom wrapper for MoveIt2 that enables simpler interaction from python scripts.


