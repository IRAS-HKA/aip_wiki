# Cheatsheet: Terminal 

This section provides a cheatsheet with all major necessary commands for the AIP application. 

## I. GitHub
   
1. Create SSH Key on PC and store on your device
2. Clone the GitHub repo with SSH method 
    ``` bash
    git clone git@github.com:IRAS-HKA/aip_coordinator.git
    ```
3. Initialize the linked submodules from the GitHub repository 
    ``` bash
    git submodule init
    ```
4. Update the initialized submodules (necessary after pushed updates to the sub repositories)
    ``` bash
    git submodule update  
    ``` 


## II. Docker
   
1. Check currently running dockers
    ``` bash
    docker ps
    ```

2. Build docker
    ``` bash
    source build_docker.sh
    ```

3. Start docker
    ``` bash
    source start_docker.sh
    ```

4. Start already running docker (e.g. in further terminals)
    ``` bash
    docker exec -it <docker_name> bash
    ```

5. Selective build of the software packages 
    ``` bash
    colcon build --packages-select <package_name>
    ```


## III. Start Up for Kuka KR10 Robot

1. Establish communication 
   - KUKA HMI (SmartPad) must be set in AUT-Mode for operation 
   - EKI_HW_Interface file must be selected and running as active project 
   - Please pay attention to the specified velocities!
 
2. Start RVIZ
   - Pay attention to the hardware flag
    - **True** = Actions will only be performed in RViz simulation 
    - **False** = Actions will be performed on the physical KR10. 
        ``` bash
        # in aip_bringup docker 
        ros2 launch aip_cell_description aip.launch.py use_fake_hardware:=false robot_ip:=10.166.32.145
        ``` 

3. Start the Bosch Gripper node to enable gripper movements 
    ``` bash
    # in aip_bringup docker 
    ros2 run aip_bosch_gripper aip_bosch_gripper_node 
    ``` 

4. Execute the gripper via service calls from the command line
   ``` bash
   # Service call for ROS node OpenGripper cylinder 1 and 2 
   ros2 service call /open_gripper iras_interfaces/srv/MoveGripper '{cylinder_ids: [1,2]}'
   ```
    ``` bash
    # Service call for ROS node CloseGripper cylinder 1 and 2 
    ros2 service call /close_gripper iras_interfaces/srv/MoveGripper '{cylinder_ids: [1,2]}'
    ```

5. Start Behaviour Tree
    ``` bash
    # in aip_coordinator docker
    ros2 launch aip_coordinator aip.launch.py

    # the to be executed behavior tree can be adjusted in the params.yaml file located in src/aip_coordinator/config/params.yaml
    ```

## IV. Miscellaneos 

1. Echo and set the ROS Domain ID
    ``` bash
    # check your currently set ROS_DOMAIN_ID  
    echo $ROS_DOMAIN_ID

    # Set ROS_DOMAIN_ID. If there are multiple robots in use, make sure, that the set ROS_DOMAIN_ID is different on each PC
    set Domain ID

    # Source workspace
    source install/setup.bash
    ```

2. Get the service type from the service (/open_gripper or /close_gripper)
    ``` bash
    ros2 service type <service_name> 
    ```
