# Quick Start Guide

This chapter contains a quick start guide for the AIP LLM application.   
Please use the GitHub commit status with the tag **`v1.0`**.

**Disclaimer:**   
As prerequisites, it requires a running plc for safety , vaccuum system and a running EKI connection on the KUKA SmartPad. 
If you require more information, please check out the rest of the documentation. 


## Terminal 1 - Bring Up

1. Start the docker container
	```shell 
	cd ~/aip_ros2_ws/src/aip_bringup
	```
	```shell 
	. start_docker.sh
	```

2. Launch the corresponding nodes for aip_bringup
	```shell 
	ros2 launch aip_cell_description aip.launch.py use_fake_hardware:=false robot_ip:=10.166.32.145
	```
**Note**: 
- The EKI Hardware interface must be running on the KUKA SmartPad
- If the safety system detects any potential risk and stops the system, the system needs to be acknowledged and restarted (EKI programm on KUKA SmartPad & Terminal for aip_bringup)


## Terminal 2 - Bosch Gripper

1. Connect to the running aip_bringup docker container 
	```shell 
	docker exec -it aip_bringup bash 
	```

2. Launch the gripper node

	```shell 
	ros2 launch aip_bosch_gripper aip_bosch_gripper_node.launch.py only_visualize:=false 
	```
**Note**: 
- If `only_visualize:=true` then the bosch gripper controller node won't be started. 
- This can be used if running AIP virtually only in Rviz

## Terminal 3 - Roboception driver

1. Start the docker container 
	```shell 
	cd ~/aip_ros2_ws/src/roboception_driver
	```
	```shell 
	. start_docker.sh
	```
The driver will be started automatically. 



## Terminal 4 - Object Detector Tensorflow (ODTF)

1. Start the docker container 
	```shell 
	cd ~/aip_ros2_ws/src/object_detector_tensorflow
	```
	```shell 
	. start_docker.sh --no-gpus
	```
	If you want to use the the GPU hardware performance, please delete the flag `--no-gpus` from the shell command. 

2. Launch the detection nodes
	```shell 
	ros2 launch object_detector_tensorflow detection.launch.py
	```
To show the odtf result manually, it is possible to use the client with:
	
```shell
	ros2 run object_detector_tensorflow client
```

## Terminal 5 - Grasp Planning

1. Start the docker container 
	```shell 
	cd ~/aip_ros2_ws/src/aip_grasp_planning
	```
	```shell 
	. start_docker.sh
	```

2. Launch the grasp_planning nodes
	```shell 
	ros2 launch aip_grasp_planning grasp_planning.launch.py 
	```
	If running the launch file via VS-Code debugging functionality, please additionally use:    

	```shell 
	ros2 launch point_transformation point_transformation.launch.py
	```

## Terminal 6 - LLM Back End

1. Start the docker container 
	```shell 
	cd ~/aip_ros2_ws/src/LLM_Scene_Docker
	```
	```shell 
	. start_docker.sh
	```

## Terminal 7 - Website (via LLM Docker)

1. Connect to the running aip_bringup docker container 
	```shell 
	docker exec -it llm_docker bash 
	``` 

2. Build and source
	```shell 
	colcon build --symlink-install && source install/setup.bash
	``` 
	
3. Launch the website node
	```shell 
	ros2 launch pkg_website_llm launch_all_services.launch.py
	``` 

How to access the website for user interaction?
- Open your desired browser 
- Navigate to the following local host URL: http://127.0.0.1:8080/

## Terminal 8 - Packing Planning

1. Start the docker container 
	```shell 
	cd ~/aip_ros2_ws/src/aip_packing_algorithm 
	```
	```shell 
	. start_docker.sh
	```

2. Launch the pack server node
	```shell 
	ros2 run pkg_pack_node pack_server 
	```

## Terminal 9 - Coordinator

1. Start the docker container 
	```shell 
	cd ~/aip_ros2_ws/src/aip_coordinator 
	```
	```shell 
	. start_docker.sh
	```

2. Start the Behavior Tree sequence via the launch file
	```shell 
	ros2 launch aip_coordinator aip.launch.py
	```



