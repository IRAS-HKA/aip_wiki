# How to setup the laboratory pc

This sections provides a quick setup guide for your laboratory pc and specific reqirements for the AIP application.

 
## Basic installation
1. Verify that your pc has a working network and wireless LAN connection to access the internet and to connect to the robot <br>
    **Note:** We recommend to use a pc with a SSD, since developing on a hard drive is slow.
2. Install Linux System ( e.g. Ubuntu 20.04, see [Link](https://releases.ubuntu.com/focal/))
3. Install Docker environment ([Link](https://docs.docker.com/engine/install/))
4. Install ROS Humble ([Link](https://docs.ros.org/en/humble/Installation.html))

## AIP specific requirements

1. Sign in to KA-Wlan via wireless LAN 
2. Make sure to have a valid GitHub account (with activated 2FA) and access to the repo: [IRAS-HKA/aip_wiki](https://github.com/IRAS-HKA/aip_wiki.git)
3. Clone the required GitHub repositories. 
   **Note:** For more information, please review the specific AIP setup guide (["How to start AIP"](/docs/how_to_start_aip.md)).
4. Connect the KUKA Robot System to LAN connection of your PC LAN <-----> X66
5. Test your connection by sending a ping to robot through your terminal
```shell
ping 10.166.32.145
```