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

### Manual mode via the KUKA SmartPad 

**=> ToDo: Add the file names to be executed + stored location on the pad:**
- active project 
- 


For more information please see the  **_ready2_educate KUKA documentation_**


### Automatic modus via EKI (RViz)

Further information can be found in the corresponding repository: [IRAS-HKA/aip_bringup](https://github.com/IRAS-HKA/aip_bringup.git). 
Especially, the ReadMe provides detailled information regarding the start up. 

1. Prerequisite: Docker and connection to robot is up and running
   - Clone and build repository "AIP Bringup" 
     - Open a terminal (Ctrl + Alt + T) and navigate to project folders 
        ```shell
        mkdir -p ~/projects && cd ~/projects
        ```

     - Clone repo (if not already cloned); log in with your RZ account 
        ```shell
        git clone -b dev https://www.w.hs-karlsruhe.de/gitlab/iras/common/instructions/iras_robots/aip_bringup.git
        ```

     - Navigate to cloned repo 
        ```shell
        cd aip_bringup
        ```
     - Build container 
        ```shell
        ./start_docker.sh
        ```
        If everything went well, you should be in the container like this: `robot@IRAS-IRL0-LIN:~/ros_ws$`
      - Set ROS_DOMAIN_ID. If there are multiple robots in use, make sure, that the set ROS_DOMAIN_ID is different on each PC
        ```shell
        export ROS_DOMAIN_ID=<id>
        ```
      - You can check your currently set ROS_DOMAIN_ID by running
        ```shell
        echo $ROS_DOMAIN_ID
        ```
      - In the container, build workspace:
        ```shell
        colcon build
        ```
      - Source workspace
        ```shell
        source install/setup.bash
        ```

2. Connect with second terminal to existing docker session
    - Execute the still running container
        ```bash
        docker exec -it aip_bringup bash
        ```
    - As you´ve opened a new terminal, you need to set your ROS_DOMAIN_ID again, use the same ID as before
        ```bash
        export ROS_DOMAIN_ID=<id>
        ```
    - You can check your currently set ROS_DOMAIN_ID by running
        ```bash
        echo $ROS_DOMAIN_ID
        ```
    - Source workspace
        ```bash
        source install/setup.bash
        ```
    - Launch robot driver with MoveIt2 wrapper
        ```bash
        ros2 launch kuka_kr3_cell_description cell.launch.py
        ```
        This will open up a simulated hardware with visualisation.
        To launch the real robot:
        - Make sure that you are in our local network (Wi-Fi or LAN)
        - Test your application in simulation first
        - Make sure that the robot is not in a collision state when the application is executed on the real robot
        - If everything is fine, execute
        ```bash
        ros2 launch kuka_kr3_cell_description cell.launch.py use_fake_hardware:=false robot_ip:=<robot-ip>
        ```

3. Check out tutorial code 
   - For the tutorial code, please check out the repository [IRAS-HKA/aip_bringup](https://github.com/IRAS-HKA/aip_bringup.git), if you haven´t already. 

4. Move Robot 
   - Open up a new terminal and attach to running container
        ```bash
        docker exec -it r2e_cell /bin/bash
        ```
    - If you have opened a new terminal you need to set your ROS_DOMAIN_ID again, use the same ID as before
        ```bash
        export ROS_DOMAIN_ID=<id>
        ```
    - You can check your currently set ROS_DOMAIN_ID by running
        ```bash
        echo $ROS_DOMAIN_ID
        ```
    - Source workspace
        ```bash
        source install/setup.bash
        ```
    - Run same application
        - before running the application, check the robot's movement in the simulated environment and make sure, that the robot is not in a collision state when the application is executed on the real robot
        ```bash
        ros2 run r2e_demos test_ros_env
        ```
   
5. Move Robot via EKI and RVIZ 
- Start Rviz in second terminal of your running docker container
    ```bash
    ros2 run rviz
    ```
- Use mouse courser to drag robot to your desired position
- Click "Plan" in order to plan the trajectory in the simulation 
- Click "Plan & Excecute" in order to move the robot and in the simulation
  - Pay attention: If you´ve set the parameter use_fake_hardware:=false, the action will be executed on the real robot. Make sure that it won´t trigger a collision

### 3. Automatic modus via behaviour tree

For more information, please review the following repository: [IRAS-HKA/aip_coordinator](https://github.com/IRAS-HKA/aip_coordinator.git)

**=> ToDo: Add explicit links and a short summary from the other repository**

### 4. Automatic modus via Python

1. Prerequisite: Docker and connection to robot is up and running
2. Connect with second terminal to existing docker session
```bash
docker exec -it aip_bringup bash 
```
3. In the second terminal you can execute the following python scripts in order to move the robot basend on coordinates. Example python file.
```bash
ros2 launch aip_cell_description aip.launch.py
```
**=> ToDo**




# TODO

- [ ] Add manual mode description 
- [x] Add EKI/ RViz description
- [ ] Add automatic modus behaviour tree description