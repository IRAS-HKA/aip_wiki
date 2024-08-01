# Packing Algorithm of AIP

This section provides information about the packing algorithm that can be used on the AIP demonstrator. The original algorithm was developed by Matthias Reiplinger in his master thesis in and has been improved several times. Basis of the now implemented version of the packing algorithm is the bachelor thesis of Luisa Schlenker.

Since the packing algorithm was is originally written in a Jupyter notebook, the script was organised in several classes and interfaces were written in order to enable information transfer between classes and use it in combination with the existing Behavior Tree and ROS2. No modifications of the core algorithm were made, with two exceptions: the maximum container number of containers to be packed was limited to one. And the visualization was changed to be able to export an image of the calculated container. The transfer of information is no longer being done with Excel sheets or csv files, but now with dataframes. Getter-Setter methods enable the transfer of variables and dataframes between different classes. The material master is implemented using a YAML file.

Due to reasons of clear task seperation, some parts of Luisa Schlenker's original work, such as the calculation of the robot's grasp position, were transferred to the grasp planning part.

## Procedure of the actual Implementation

In order to start the calculation of the optimal packaging sequence and ideal volume utilization, the ROS2-server "*PackAlgorithm_Server*" needs to receive a list of strings of objects to be packed. During the normal process, this is provided by the LLM.

The service starts the algorithm with the "Pack-Algorithm.py file. The algorithm then uses the objects from the LLM to calculate the "Packplan". For further information of procedure of the exact calculation, see the work of [Matthias Reiplinger](https://hskarlsruhede.sharepoint.com/:b:/s/Robolab/EZF0UL3QLYBPpvjeM2Y3UkwB81SG-OiJ-25jvLSR6Ph6Bw?e=szWvIF), [Dominik Lipfert](https://hskarlsruhede.sharepoint.com/:b:/s/Robolab/EWJFCraKGNpHjp71pGM5PiwB4MVVkDBT9mjcQsUsD9rr4w?e=l5ZdA5), [Anton Schulz](https://hskarlsruhede.sharepoint.com/:b:/s/Robolab/EZNjMJRuXIdHtjaKTe_28R8BG21SULXIZlKrc7NizC40tw?e=69PM1x) and [Luisa Schlenker](https://hskarlsruhede.sharepoint.com/:b:/s/Robolab/Ec22It_Fk9hKm28NS1hbQX4BLJKMpIRFVnF3OMU-dXNE-Q?e=IMnUJT). This [Ilias link](https://ilias.h-ka.de/goto.php?target=wiki_342982_Pack-Algorithmen#il_mhead_t_focus) provides more information on previous versions of the packing algorithm.

The output of the packing algorithm is the packplan. It is organized in a ROS2 message and contains the following information for each package:

- Class name
- Package dimensions
- Weight
- Rotation index
- Place coordinates

The packages in the packplan are arranged in the optimal packaging sequenceand is processed into the needed format in the "*Packplan_Processing.py*" script. Furthermore, a visualization of the optimal finished container is made and sent back to the LMM via a topic. An example of the visualization can be seen in the following.

<img src="../images/20240729_solution_screenshot.png" width="900"/>

## How to start the PackAlgorithm server for test purposes

1. Start server in Terminal
    - Start Docker
    - Execute the following command:

    ```shell
    ros2 run pkg_pack_node pack_server
    ```

2. Execute a service call
    - Connect to the running container with the following command:

    ```shell
    docker exec -it aip_packing_planning bash
    ```

    - Execute the service call:

    ```shell
    ros2 service call pack_planning aip_packing_planning_interfaces/srv/PackSequence '{}'
    ```

3. Listen to SolutionFeedback Publisher
    - Connect to the running container with the following command:

    ```shell
    docker exec -it aip_packing_planning bash
    ```

    - Listen to the topic "*solution_feedback*" with the following command:

    ```shell
    ros2 topic echo solution_feedback
    ```

**Important**: For the simulation the following line in the "*PackAlgorithm_Server*.py" file must be **active**. Here the items to be packed can be typed in.

```shell
items = ["Box_Gluehlampe", "Box_Wischblatt", "Keilriemen_gross"]
```

The following line mus also be **deactivated**:

```shell
items = request.objects_to_pick
```

## Changes in comparison to original Packing Algorithm in a Nutshell

- B1.Packalgorithmus and example.py (only relevant things) merged.
- B1.Packalgorithmus separated into classes.
- Dataframe of packing list and container used transferred instead of csv. Getter-Setter method used for this.
- Getter-Setter methods created for transfer of required variables.
- Material master integrated as YAML.
- Container dimensions hard coded. Currently only one container is available. (Changes in Container.py)
- Service written to receive objects to be packed and transfer packing plan
- Visualisation adapted (in Container.py). Visualisation is saved in image and no longer displayed.
- Saved image is published via topic
- Container quantity limited to one. All containers are calculated, but only the first one is transferred and displayed (changes in solution.py)
