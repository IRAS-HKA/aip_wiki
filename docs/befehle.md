BringUp:
0. Docker startebn
1. source install/setup.bash
2. EKI Hardware interface am KUKA smart pad starten
2. ros2 launch aip_cell_description aip.launch.py use_fake_hardware:=false robot_ip:=10.166.32.145
3. docker exec -it aip_bringup bash (in neuem Terminal auf Host)
3. ros2 run aip_bosch_gripper aip_bosch_gripper_node für Gripper server

Roboception driver
1. Docker container starten

ODTF:
1. Docker staren
2. ros2 launch object_detector_tensorflow detections.launch.py

aip_grasp_planning:
1. Docker starten

LLM Docker:
1. Docker starten

aip_pack_planning
1. Docker starten
2. ros2 run pkg_pack_node pack_server
Funktioniert nicht ganz iwie import fehler

User interface website:
1. Docker starten

aip_coordinator:
1. Docker starten
2. source install/setup.bash
3. ros2 launch aip_coordinator pipeline_test.launch.py

