# Cheatsheet: Terminal 

This section provides a cheatsheet with helpful commands for the AIP application. 

For more explicit information to the start of the AIP application, please check the [AIP_BringUp GitHub Repository](https://github.com/IRAS-HKA/aip_bringup) and [How to start AIP](/docs/how_to_start_aip.md).


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
## III. Miscellaneos 

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
