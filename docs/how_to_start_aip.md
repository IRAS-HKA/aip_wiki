# How to start the AIP application

This sections provides a quick guide to start the AIP application. It enables you to run the complete AIP system and engineer your extended application.

For more information, please also review the corresponding AIP MS Teams Team:
[**_IRAS Students/General/01-IRAS_Wiki/2-Projects/2.1-Automated_Item_Picking_**](https://hskarlsruhede.sharepoint.com/:f:/s/Robolab/EqgV9DKqqRJDrYVzu5INeNgBFf0JPXn-Eccabwk7Z6qXew?e=N3B4xT)

## I. Quick start guide

1. Plug in the cabinet and power all devices
2. Check the PLC status
3. Check the KUKA SmartPad status
4. Start your docker environment on the laboratory PC

Notes:

- _The ejectors switch while the PLC is starting up. If there is compressed air in the system at the same time, this leads to a loss of pressure and a noticeable noise. However, this is not dramatic. As soon as the PLC has started up, the ejectors are set to their normal state and the pressure loss ends._

  _To prevent this behaviour, the air supply on the Festo maintenance unit can be closed._
- _If the PLC is not starting properly, the 7 days trial license may be outdated. To verify, please open the electrical cabinet and check if the PLC is in run mode (green light) or in configuration mode (blue light). If the blue light is on, this can be an license issue. For a quick solution, please check the assigned [trouble shooting](/docs/troubleshooting.md) of this repository._
- _To use the system after it has been switched on, the blue button on the control panel must be pressed. This does not light up at the beginning. Then the safety release has then been given and the robot can be used._
  
## II. Different movement options

There are 4 different options to operate the AIP application which will be described in the following subchapters.

### Manual mode via the KUKA SmartPad

1. Switch to user group _Administrator_ on smartHMI (pw: kuka)
2. Make sure to clean all errors, shown in green rectangle in the picture below
3. If the error _"NOT-Halt nur lokal"_ occurs and red LED on the PLC CPU is blinking, probably the PLC is not in RUN mode.
4. Make sure that the drives are switched on
5. Make sure to set KUKA KR 10 into T1 mode
6. Push the enabling switch of the SmartPad while moving an axis with the buttons, marked with the red rectangle in the picture below, or the 6D mouse.

    <img src="../images/KUKA_SmartPad.jpeg" width="600"/>

### How to operate the Gripper

To operate the gripper manually with the SmartPad, please check[How to use KUKA SmartPad](/docs/how_to_use_kuka_hmi.md).

### How to set robot in EKI mode

The robot has been set into EKI mode in order to listen to the Linux-PC.

**Import**: If you´re not familiar with ROS2 and AIP commands, please check out the [Cheatsheet with terminal commands](/docs/cheatsheet_terminal.md) for quick help. 

1. Switch to user group Administrator on SmartPad
2. Activate project "ros2_driver" on SmartPad, if not already active
    - Open project management window (Blue WorkVisual icon, gear with robot inside)
    - Select "ros2_driver" in Verfügbare Projekte → Entpinnen
    - "Aktivieren" → Ja
    - Wait until project is activated
3. On SmartPad navigate to R1 → Program → ros2_driver
4. Load the file "kuka_eki_hw"
5. Click on "Abwählen" to deactivate the current programm
6. Click on "Anwählen" to activate the "kuka_eki_hw"

    <img src="../images/KUKA_EKI_HW.png" width="600"/>

7. Execute the programm
8. During the execution you can see the following code:

    <img src="../images/CodeKUKA.png" width="600"/>

### Automatic modus via EKI (RViz)

Further information for this operating mode can be found in the corresponding repository: [IRAS-HKA/aip_bringup](https://github.com/IRAS-HKA/aip_bringup.git). Especially, the ReadMe provides a brief overview for the start up.

1. Prerequisite: Docker and connection to robot is up and running
    - Clone and build repository "[AIP Bringup](https://github.com/IRAS-HKA/aip_bringup.git)"
    - Open a terminal (Ctrl + Alt + T) and navigate to project folders

        ```shell
        mkdir -p ~/projects && cd ~/projects
        ```

    - Clone repository, if not already cloned. Log in with your RZ account.

        ```shell
        git clone https://github.com/IRAS-HKA/aip_bringup.git
        ```

    - Navigate to cloned repository

        ```shell
        cd aip_bringup
        ```

    - Build container

        ```shell
        ./start_docker.sh
        ```

        If everything went well, you should be in the container like this:

        `robot@IRAS-IRL0-LIN:~/ros_ws$`

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

2. Start launch file

    - Execute the still running container

        ```bash
        docker exec -it aip_bringup bash
        ```

    - As you have opened a new terminal, you need to set your ROS_DOMAIN_ID again, use the same ID as before

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
        ros2 launch aip_bringup aip.launch.py use_fake_hardware:=true robot_ip:=10.166.32.145
        ```

        This will open up a simulated hardware with visualisation.

        To launch the real robot:
        - Make sure that you are in the local network (Wi-Fi AND LAN)
        - Test your application in simulation first (see above)
        - Make sure that the robot is not in a collision state when the application is executed on the real robot
        - If everything is fine, please execute:

       ```bash
        ros2 launch aip_bringup aip.launch.py use_fake_hardware:=false robot_ip:=10.166.32.145
        ```

      **Pay attention**: If you´ve set the parameter use_fake_hardware:=false, the action will be executed on the real robot. Make sure that it won´t trigger a collision.


   - Use mouse courser to drag robot to your desired position
   - Click "Plan" in order to plan the trajectory in the simulation
   - Click "Plan & Excecute" in order to move the robot and in the simulation

3. Check out tutorial code

   - Please check out the repository [IRAS-HKA/aip_bringup](https://github.com/IRAS-HKA/aip_bringup.git), if you haven´t already.


### 3. Automatic modus via Behaviour Tree

The information for how to move the robot using a behavior tree are described in the repository [IRAS-HKA/aip_coordinator](https://github.com/IRAS-HKA/aip_coordinator.git) and the [How to use AIP Coordinator](/docs/how_to_use_aip_coordinator.md) of this repository.

The "IRAS Coordinator" package provides a starting point for high-level task control of the AIP robot application.

The library of actions can be freely arranged using the "Groot" graphical user interface. There are instructions on how to create and start the Docker container as well as instructions on how to start debugging.

Once the container is started, the description can be used to design a new or change the behavior tree in Groot.  
Furthermore, instructions are given on how to create a new custom node and how to create behavior trees.
To control the gripper, we have created two services: /open_gripper and /close_gripper.
Both are using the iras_interfaces/srv/MoveGripper-datatype.

The only input is cylinder_ids, which an array of int32. The content of the array are the ejectors, which will be opened.
Those are being read from the behavior tree.

#### 3.1. Start automatic mode via Behavior Trees

1. Prerequisite: Docker (AIP_BringUP) and connection to robot is up and running.
2. It is recommended to use Terminator to use many terminal windows in parallel.
3. Connect with second terminal to existing docker session:

    ```bash
    docker exec -it aip_bringup bash 
    ```
    Connect in first terminal with the robot:

    ```bash
    ros2 launch aip_bringup aip.launch.py use_fake_hardware:=false robot_ip:=10.166.32.145 
    ```
    
    ```
    **Note:** If `use_fake_hardware:=true` then the gripper controller node will not be started. This can be done when running AIP only virtually in Rviz.

4. In the third terminal you can execute the following python scripts in order to move the robot based on coordinates. This needs to be executed in the AIP_Coordinator.

    ```bash
    (IF AIP_Coordinator docker is not already running) 

    cd ../aip_coordinator
    source start_docker.sh 

    (IF AIP_Coordinator docker is running) 
    docker exec -it aip_coordinator
    ros2 launch aip_bringup aip.launch.py
    ```
    This will execute the behavior tree using groot.
5.  To edit the tree switch to the edit mode and save the tree afterwards.
6.  This will take effect on the next start:
   
    ```bash
    ros2 launch aip_bringup aip.launch.py
    ```

### 4. Control robot via terminal

1. Open gripper (AIP_Coordinator/ AIP_BringUP)
   
    ```bash
    * works in both dockers *
    ros2 service call /open_gripper iras_interfaces/srv/MoveGripper '{cylinder_ids: [1,2]}'
    ```

2. Close gripper (AIP_Coordinator/ AIP_BringUP)
   * works in both dockers *
    ```bash
    ros2 service call /close_gripper iras_interfaces/srv/MoveGripper '{cylinder_ids: [1,2]}'
    ```
3.  Cancel robot operation
    Use Strg+ C to cancel the terminal which is running the command:

    ```bash
    ros2 launch aip_bringup aip.launch.py use_fake_hardware:=false robot_ip:=10.166.32.145 
    ```

4.  Disconnect the gripper but not the robot:
    Use Strg+ C to cancel the terminal which is running the command:

    ```bash
    ros2 launch aip_bosch_gripper aip_bosch_gripper_node.launch only_visualize:=false 
    ```
