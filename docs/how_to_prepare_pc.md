# How to setup the pc

## Basic Installations
1. Double check your pc has network and wireless Lan to access the internet and to connect to robot
1b. We recommend to use a pc with a SSD, since developing on a hard drive is slow.
2. Install Linux System (e.g. Ubuntu 20.04) (https://releases.ubuntu.com/focal/)
2. Install Docker environment (https://docs.docker.com/engine/install/ )
3. Install ROS Humble (https://docs.ros.org/en/humble/Installation.html)

## AIP specific
1. Sign in to KA-Wlan via wireless lan 
2. Make sure you have an valid github account (with 2FA activated) and access to the repo. 
3. Clone the required Github repositories. ->  Separate guide in "how to setup aip"
4. Connect KUKA Robot System to LAN connection of your pc LAN <-----> X66(NAME OF PORT der Steuerung!!!) PRÜFEN
5. Test connection by sending a ping to robot
```shell
ping 10.166.32.145
```