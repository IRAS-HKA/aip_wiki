# Large Language Model

This section contains information on the language model of the "Autoamted Item Picking" system. The general structure of the model is described, as well as the integration with Docker and ROS2, followed by an overview of how to operate the LLM with various commands.

## Description of LLM (Überschrift kann man ändern)

Beschreibung wie LMM aufgebaut ist und in unserem Fall funktioniert

Erwähnen dass das LLM in deutscher Sprache bedient werden muss

## LLM with ROS2

Beschreibung des Containers und der ROS Integration

The LLM container is running as a ROS2 Node and contains a Ollama installation with Mistral as LLM. The container also hosts the website where the user interaction takes place.

## Description of Website

As soon as the container has been started, the website can be accessed. It is hosted locally and is available via the following URL:

```shell
http://localhost:8080/
```

### Overview of Website

The website contains several elements that also provide an overview of the current status of the machine.

On the left-hand side is an image of the current depth camera readings. In the middle of the page is the result of the packing algorithm.It shows the calculated container as it will be packed by the robot. On the right-hand side is the LLM's interaction panel. This is where the chat is displayed and where commands can be entered.
The packing sequence of the package and the cylinder IDs are displayed below the left image. The currently running nodes are displayed below the image of the calculated container. This panel is primarily used to analyse the whole system.

# Bild aktualisieren!!!!!!!

<img src="../images/20240808_Website.png" width="1000"/>

### Interaction and possible Commands

The chat with the LLM offers various options for interaction. Different chat modes are available via a drop-down selection in the bottom right-hand area of the chat window.

#### Regular Chat

This mode is selected by choosing "**Chat**". Normal chat is possible here. The LLM has no scene understanding of the camera or associated object information. For example, a question can be asked about how to repair a car.

#### Chat including additional Object Information

This mode can be selected with "**Scene chat**" (Vielleicht Wort ändern). In this mode, the LLM has an understanding of the scene and background information on the respective objects from the material master. Specific information, such as the position in the scene or the weight of an object, can be queried. A typical input could be: Tell me everything you know about the object in the centre of the scene.

#### Command

This mode is selected with "**Befehl**". It should be used when objects are to be transferred to the packing algorithm. In this mode a JSON object is created in the background with the objects to be packed. The objects are specified in the chat. If the selection is to be packed, this must be confirmed with the 'Bestätigen' button. The objects are then passed to the packing algorithm. The displayed selection can alternatively be rejected with the "Ablehnen" button. A typical command might be Pack part X and part Y.

## How to run the LLM container

To run the following commands, the Docker container must already be running. To start the Docker container, clone the LLM repository and navigate to the LLM_Scene_Docker folder. Launch the docker using the following terminal command:

```shell
source start_docker.sh
```

Once the container is running open a new terminal and connect to the running container with the following command.

```shell
docker exec -it llm_docker bash
```

Then run one of the following commands.

### Start Website and LLM via Launchfile *(preferred option)*

```shell
cd && cd ros_ws && colcon build && source install/setup.bash && cd src/pkg_website_llm && cd launch && clear && ros2 launch launch_WebsiteAndLLM.py
```

### Start only Website

```shell
cd && cd ros_ws && colcon build && source install/setup.bash && cd src/pkg_website_llm && cd launch && clear && ros2 launch launch_only_Website.py
```

### Start only LLM

```shell
cd && cd ros_ws && colcon build && source install/setup.bash && cd src/pkg_website_llm && cd launch && clear && ros2 launch launch_only_LLM.py
```

### Start only UserInputService

```shell
cd && cd ros_ws && colcon build && source install/setup.bash && cd src/pkg_website_llm && cd launch && clear && ros2 launch launch_only_FeedbackWebsite.py
```

Once the container is started, the entry point is directly on the cli of ollama.

### Call Ollama Python API test

Navigate to the follwing folder:

```shell
llm_scene_docker/llm_files/
```

Execute the following command:

```shell
python3 MainLLM.py
```

## How to request the User Input

1. Connect to Docker
2. Run the following commands

    ```shell
    ros2 service call /user_interaction llm_interfaces/srv/UserInteraction {''}
    ```

    ```shell
    ros2 service call scene_interpretation llm_interfaces/srv/SceneInterpretation "{user_input: 'TEST'}"
    ```

Subsequently the terminal shows the user input.

## *For Debugging:* How to start the Action Client and Server to send the user input to the LLM

### Client

1. Open New Terminal
2. Connect to LLM_Docker
3. Run the command

    ```shell
    colcon build && source install/setup.bash
    ```

4. Navigate to the folder using:

    ```shell
    cd src/pkg_website_llm/pkg_website_llm/
    ```

5. Run the command

    ```shell
    python3 ActionClientToPreProcessing.py
    ```

### Server

1. Open New Terminal
2. Connect to LLM_Docker
3. Run the command

    ```shell
    colcon build && source install/setup.bash
    ```

4. Navigate to the folder using:

    ```shell
    cd /src/pkg_llm_docker/pkg_llm_docker
    ```

5. python3 LLM_Action_Server.py

With the following command you can send a test request to the LLM

```shell
ros2 action send_goal /llm_action_server llm_action_interfaces/action/LLM "{userinput: 'BEFEHL: Box_Wischblatt' }"
```

## How to mock (bypass the LLM)

1. Connect to Docker
2. enter on the terminal:

    ```shell
    ros2 run pkg_pack_item_server pack_item_server  
    ```

3. Result is hard coded:

    ```shell
    ['Box_Gluehlampe', 'Box_Wischblatt','Keilriemen_gross', 'Box_Bremsbacke', 'Keilriemen_klein', 'Tuete']
    ```

## How to start the action server of the LLM

With the follwong command it is possible to start the action server of the LLM.

```shell
ros2 action send_goal /llm_action_server llm_action_interfaces/action/LLM "{userinput: "Box_Wischblatt"}"
```
