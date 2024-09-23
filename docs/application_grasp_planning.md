# AIP Grasp Planning

This repository contains the 6-DoF Grasp Planning for the Automated Item Picking (AIP) application at the University of Applied Sciences in Karlsruhe (HKA).

## Operating principle

The grasp planning is based on the following operations: 
  1. Preprocessing   
     - Receive depth image from the Roboception camera  
     - Transform the depth image based on the object mask from ODTF into a point cloud with a resolution of 10mm   
  2. Determination of the 6-DoF grasp poses   
     - Filtering the point cloud
     - Extraction of the object surface with RANSAC algorithm including projection of all points onto the determined surface    
     - Determination of the normal vector for the surface    
     - Determination of the grasp point as median of all relevant points in the point cloud    
     - Determination of the grasp vector by including the orientation per package received from ODTF

      <div style="display:flex;">
        <img src="aip_grasp_planning/docs/pcl_image1.png" width="350">
        <img src="aip_grasp_planning/docs/pcl_image3.png" width="350">
      </div>
      <br>

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

## Interface diagram

<img src="aip_grasp_planning/docs/aip_grasp_planning.drawio.png" width="1000"><br>

| Inputs | Outputs |
|--------|---------|
| - Objects to Pick and Place <br> - Package Sequence for placement planning <br> - Depth image <br> - Detections from ODTF | - Pick poses <br> - Cylinder IDs to grasp the objects <br> - Place poses        |

The necessary interfaces from ODTF and Packing Planning are cloned from the most recent GitHub status of the main repositories per module (see Dockerfile).  Consequently, they are only visible after attaching to the running container.
If you need to adapt the interfaces, please change them in the repository of the modules and rebuild the docker image.

For more information on the necessary inputs and outputs, please review the module repositories: 
- [Object_Detector_Tensorflow](https://github.com/eshan-savla/object_detector_tensorflow) 
- [AIP_Packing_Algorithm](https://github.com/SchmittAndreas/aip_packing_algorithm) 

## Visualization

Grasp poses and the resulting object surface point cloud can be visualized in RVIZ. Therefore the following topics are published:

- /grasp_object_surface_point_cloud (White PointCloud2)
- /grasp_poses (Green arrow)
- /grasp_poses_with_offset (Red arrow)

<img src="aip_grasp_planning/docs/rviz.JPG" width="400"><br>

## Miscellaneous

Launch the point transformation node for additional debugging:

```bash
ros2 launch point_transformation point_transformation.launch.py 
```




