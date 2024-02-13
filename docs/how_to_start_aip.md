# How to start the AIP application?

This sections provides a quick guide to start the AIP application. It enables you to run the complete AIP system and engineer your extended application.

For more information, please also review the corresponding AIP MS Teams Team: 
**_IRAS Students/Projects/.-Automated_Item_Picking_**


## I. Quick start guide 

1. Plug in the cabinet and power all devices
2. Check the plc status 
3. Check the KUKA SmartPad status
4. Start your docker environment on your laboratory PC 


## II. Different movement options

There are 4 different options to move the AIP application which will be described in the following subchapters.

### 1. Manual mode via the KUKA SmartPad 

For more information please see the  *_ready2_educate KUKA documentation_*

### 2. Automatic modus via RViz 

1. Prerequisite: Docker and connection to robot is up and running
2. Connect with second terminal to existing docker session
```bash
{docker exec -it <container_id_or_name> bash }
```
3. Start Rviz in second terminal of your running docker container
4. Use Mouse Courser to drag robot to the position you like
5. Click "Plan & Excecute" in order to move the robot in real life and in the simulation

### 3. Automatic modus via behaviour tree

For more information, please review the following repository: [IRAS-HKA/aip_coordinator](https://github.com/IRAS-HKA/aip_coordinator.git)


### 4. Automatic modus via Python

1. Prerequisite: Docker and connection to robot is up and running
2. Connect with second terminal to existing docker session
```bash
{docker exec -it <container_id_or_name> bash }
```
3. In the second terminal you can execute the following python scripts in order to move the robot basend on coordinates. Example python file.
```bash
{ros2 launch aip_cell_description aip.launch.py}
```
**=> ToDo**


