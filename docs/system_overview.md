# System Overview
The AIP system is modelled below. Sensors and actuators are wired to the slaves. The IO link slaves are wired to the IO link master. The master is connected to the KR10 robot control. The configuration of the IO link master is done in Kuka WorkVisual.

All network participants are connected to the switch.

The PLC is used to provide the safety-related aspects of the system. A Sick lidar scanner, an emergency stop and a signal tower are wired to the PLC. The PLC is communicating with the robot control and is giving the safety approval.

The robot can be controlled via the Kuka Smartpad or by starting a behavior tree on the computer. Alternatively RViz can be used to move the robot to a specific pose.

<img src="..\images\20231106_AIP-Overview-Detailed.png" alt="system_overview" width="1000">


Below the detailed view of the IO Link wiring is shown.

<img src="..\images\20231106_Detailansicht IO Link.png" alt="system_overview" width="1000">


