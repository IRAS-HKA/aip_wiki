# How to setup the laboratory pc

This sections provides a quick setup guide for your laboratory pc and specific reqirements for the AIP application.

## Basic installation

1. Verify that your PC has a working network and wireless LAN connection to access the internet and to connect to the robot

    Note: _It is recommended to use a PC with a SSD, since developing on a hard drive is slow._

2. Install Linux System (e.g. Ubuntu 20.04, see [Link](https://releases.ubuntu.com/focal/))
3. Install VS Code ([Link](https://code.visualstudio.com/download))
4. Install Docker environment ([Link](https://docs.docker.com/engine/install/))
5. Install ROS Humble ([Link](https://docs.ros.org/en/humble/Installation.html))

## AIP specific requirements

1. Sign in to WLAN
2. Make sure to have a valid GitHub account (with activated 2FA) and access to the repository: [IRAS-HKA/aip_wiki](https://github.com/IRAS-HKA/aip_wiki.git)
3. Clone the required GitHub repositories.

   Note: _For more information, please review the specific AIP setup guide (["How to start AIP"](/docs/how_to_start_aip.md))._

4. Connect the KUKA robot control via socket X66 with LAN connection to your PC
5. Test your connection by sending a ping to robot control using your terminal

    `ping 10.166.32.145`
