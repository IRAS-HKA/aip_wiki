# Cheatsheet: Terminal 

Clone with SSH from GitHub
1. Create SSH Key on PC 
2. git clone git@github.com:IRAS-HKA/aip_coordinator.git
3. git submodule init
4. git submodule update


Build docker
``` bash
source build_docker.sh
```

Start docker
``` bash
source start_docker.sh
```

Start running docker
``` bash
docker exec -it <docker_name> bash
```

Rviz Starten in AIP Bringup => Pay attention to the hardware flag !!
KUKA HMI must be in AUT-Mode and EKI_HW_Interface must be running!

``` bash
ros2 launch aip_cell_description aip.launch.py use_fake_hardware:=false robot_ip:=10.166.32.145
```

Start Behaviour Tree => opens Groot 
``` bash
ros2 launch aip_coordinator aip.launch.py
```

Start Bosch gripper node 
``` bash
ros2 run aip_bosch_gripper aip_bosch_gripper_node 
```

Service call for the ROS node OpenGripper/CloseGripper Cylinder 1 and 2
``` bash
ros2 service call /open_gripper iras_interfaces/srv/MoveGripper '{cylinder_ids: [1,2]}'
```
``` bash
ros2 service call /close_gripper iras_interfaces/srv/MoveGripper '{cylinder_ids: [1,2]}'
```

Get service type 
``` bash
ros2 service type /open_gripper 
```
``` bash
ros2 service type /close_gripper 
```

Selective package build
``` bash
colcon build --packages-select aip_bosch_gripper
```
ROS2 General

``` bash
set Domain ID
echo $ROS_DOMAIN_ID
source install/setup.bash
```
