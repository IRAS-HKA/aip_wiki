# Repository Overview

This section provides a general overview of the used repositories for the AIP application.

## 1. AIP_Coordinator

This docker container handels the control via the **Groot Behavior Tree System**.
You can design new behavior trees and programm the sequence visually.
Alternatively, you can change existing behavior trees.

The full instructions and explaination of the code can be found here:
[AIP Coordinator GitHub Repository](https://github.com/IRAS-HKA/aip_coordinator)

## 2. Aip_Cell_Description

This repository contains specific description of the robot in this case KUKA KR10:
[Aip_Cell_Description GitHub Repository](https://github.com/IRAS-HKA/aip_cell_description/tree/main)

Without this repository the simulation has no representation of the robot. The repository handels the kinematics, URDF files (= description of the robots dimensions, used angels) and how to handle collisions.

If those information would be missing, the robot could crash with the environment, simply because the environment is not known.

## 3. AIP_BringUp

The full instructions and explaination of the code can be found here:
[AIP_BringUp GitHub Repository](https://github.com/IRAS-HKA/aip_bringup)

This repository explains in detail how to connect to the robot, however this contains the information of the **ready2educate cells**.

The repository can still be used, we just need to add the KUKA KR 10 parameters.
The explaination how to control the KUKA SmartPad in order to connect with ROS 2 is done in **How_To_Start_AIP**.

After cloning the repository, you can build and start the docker container.
This enables you to move the robot after initilising the robot driver. The example movement is defined in **test_ros_env**.

For the further details please take a look in the GitHub repository.

## 4. Kuka_EKI

The full instructions and explaination of the code can be found here:
[Kuka_EKI GitHub Repository](https://github.com/IRAS-HKA/kuka_eki/tree/driver-humble)

This repo contains the configurations of the KUKA EKI Interface, which is used to build up an connection between the Linux-PC and the robot control.

## 5. MoveIt Wrapper

The full instructions and explaination of the code can be found here:
[MoveIt Wrapper GitHub Repository](https://github.com/IRAS-HKA/moveit_wrapper)

Custom wrapper for MoveIt2 that enables simpler interaction from Python scripts.

## Repository Overview: Applications of AIP

In the following table, the repositories with the correct branches are listed, in order to use the AIP demonstator correctly.

| Repository Name   | Branch   | Link   |
|------------|------------|------------|
| Coordinator   | Inhalt 3   | |
| BringUp   | Inhalt 6   | |
| LMM   |  Inhalt 9   | |
| Packing Algorithm   |  Visualization   | https://github.com/SchmittAndreas/aip_packing_algorithm |
| Grasp Planning   | Inhalt 9   | |
