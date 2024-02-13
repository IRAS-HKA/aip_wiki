# WorkVisual

This sections provides a quick guide for the installation of WorkVisual as well as an overview of the project specific AIP WorkVisual project.

## I. How to install WorkVisual?

1. Download WorkVisual from the official KUKA website ([Link](https://kuka.sharefile.eu/share/view/seb15e6d0c9246e79/fofe824e-d661-457c-9858-97304df52369))
2. Install and run .exe 
3. Connect to the same subnet as the KUKA robot. For more information to the IPs, please check this [Link](/docs/devices_ips_and_passwords.md).

<img src="../images/GifWorkVisual.gif" width="900"/>

## II. Overview of the AIP WorkVisual project 

This sections provides an overview to the different components of the configured AIP WorkVisual project.
You will find the freezed project status in the corresponding MS Teams Team: 
**_IRAS Students/Projects/.-Automated_Item_Picking_**

## Configuration of PLC communication 

To set up the communication between robot control and the PLC, the EtherCAT communication interface needs to be implemented into the bus structure. The bus structure can be opened by double-clicking on the shown control (BinPicking HSKa (KRC4 compact - 8.5.5)). The necessary components are added to the KUKA Extension Bus (SYS-X44).<br>
The module KRC4 primary EL6695-1001 can be added after importing the corresponding ESI device description. Check the documentation file "KR_C4_EtherCAT_Bridge_FSoE_Master_Master_de" on MS Teams to set up the configuration of the communication <br>
Additionally, the ifm IO-Link Master AL1332 is added to the bus structure. Here also the ESI device description must be imported previously. 
For the configuration of the IO-Link master, in the tab "Modules" the following Bytes are assigned to the channels. <br>

<img src="../images/20240213_AL1332_Modules.png" width="900"><br>

## Configuration of the peripheral field modules

To link the peripheral field modules with this robot control, the in- and outputs need to be mapped. Follow the instructions in the KUKA documentation "AL1x3x_Kuka_Rev1_EN" in the "KUKA Startup Package" on MS Teams.

Open the tab "E/A Mapping" and choose the marked settings shown below in the image. By doing this the In- and Outputs if the field modules can be mapped to the KR C robot control.

<img src="../images/240213_IO_Mapping.png" width="900"><br>

Right-click on the in- or output you want to map in the lower box. Not-mapped I/Os are marked grey. As soon as the I/O is mapped it appears in the upper box. The module number represents the port/channel number on the IO-Link master.

By clicking on the pen in the bottom right corner, the signal editor can be opened. Here are the previously mapped in- and outputs shown, broken down on bit level. The names of the variables can be adapted in this view.

<img src="../images/240213_Signal_editor.png" width="900"><br>

todo: Swapping, Bild ovn allen gemappten IOs.

