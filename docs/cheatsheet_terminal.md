# Cheatsheet: Terminal 

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

Service call for the ROS node
``` bash
ros2 service call /open_gripper iras_interfaces/srv/MoveGripper '{mode: 7, open: true}'
```

Get service type 
``` bash
ros2 service type  /open_gripper 
```

Selective package build
``` bash
colcon build --packages-select aip_bosch_gripper
```



---


set Domain ID
echo $ROS_DOMAIN_ID
source install/setup.bash



git submodule init
git submodule update
