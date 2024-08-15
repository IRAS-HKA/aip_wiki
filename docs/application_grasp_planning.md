# Grap Planning

This section contains the 6-DoF Grasp Planning for the Automated Item Picking (AIP) application at the University of Applied Sciences in Karlsruhe (HKA). The corresponding repository is available via the following [Link](https://github.com/LeoSc4/aip_grasp_planning).

## Operating principle

The grasp planning is based on the following operations: 
  1. Preprocessing   
    - Receive depth image from the Roboception camera   
    - Transform the depth image into a point cloud with a resolution of 10mm   
  2. Determination of the 6-DoF grasp poses   
    - Filtering the point cloud with the received masks per object from ODTF
    - Extraction of the object surface with RANSAC algorithm including projection of all points onto the determined surface    
    - Determination of the normal vector for the surface    
    - Determination of the grasp point as median of all relevant points in the point cloud    
    - Determination of the grasp vector by including the orientation per package received from ODTF    
  3. Bosch vaccuum gripper (with 4 pneumatic individually actuated cylinders)   
    - Dynamic gripper selection based on the detected package   
    - Offset calculation to provide the correct grasp poses   

To calculate the inverse kinematics of the robot, the Trac-IK solver is configured and used in aip_bringup.   
The path planning and trajectory planning remains on the commonly used one's in MoveIt.   

## Commands

Build and start the Docker aip_grasp_planning:

```bash
source build_docker.sh
source start_docker.sh
```

Launch the nodes for grasp planning:

```bash
ros2 launch aip_grasp_planning grasp_planning.launch.py
```

## Inputs / Outputs

| Inputs | Outputs |
|--------|---------|
| - Objects to Pick and Place <br> - Package Sequence for placement planning <br> - Depth image <br> - Detections from ODTF | - Pick poses <br> - Cylinder IDs to grasp the objects <br> - Place poses        |

For more information on the necessary inputs and outputs, please review the interfaces. 


## Interfaces 

The necessary interfaces from ODTF and Packing Planning are cloned from the most recent GitHub status of the main repositories per module. (see Dockerfile)
If you need to adapt the interfaces, please change them in the repository of the modules and rebuild the docker image. 


## Miscellaneous

Launch the point transformation node for additional debugging:

```bash
ros2 launch point_transformation point_transformation.launch.py 
```




