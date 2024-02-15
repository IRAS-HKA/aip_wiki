# Explain the standard functionalities on the KUKA HWI


## How to move robot
   1. Switch to user group Administrator on smartHMI (pw: kuka)
   2. Make sure to clean all the warnings (green rectangle)
   3. If the error occurs "NOT-Halt nur lokal" and red led on the SPS is blicking, please reinstall the PLC project on the PLC 
      1. Download in MS Teams [MS TEAMS](https://hskarlsruhede.sharepoint.com/:u:/s/Robolab/EUuKVyvE1J1Phd9Jf6vTnd0B8jmQP6yJJhvDQD5b5yJrhw?e=urHCMM )
   4. Make sure to set KUKA KR 10 into T1-Mode
   5. Press the two white buttons on the back while moving the axis with the buttons (red rectangle)
        -> Robot moves now
   - <img src="../images/KUKA_SmartPad.jpeg" width="600"/>
   6. If you want to execute a programm, select it and press the "Start" button below the axis


## How to controll the gripper
   1. To read out or control sensors or actuators, the "Display" tab and then the "Inputs/Outputs" tab must be selected in the Smartpad main menu. Either the digital inputs or outputs can then be selected. The status of the inputs can be read out directly.
   2. Select Tool on the left side
   3. Press "Wert" 
   4. After the tranfers/movement select contrasting tool 
   5. Press "Wert"
   - <img src="../images/Gripper_Control.png" width="600"/>
  
## Change of Benutzergruppe
   - <img src="../images/Change_Benutzergruppe.png" width="600"/>
If options are greyed out, you can try using a different user group. The password is always "kuka".

## Change T1, T2 or Automatic Mode
Turn the key on the panel in order to switch into the menu.
  - <img src="../images/Change_of_mode_robot.jpeg" width="600"/>

  
## Minimize HMI -> Access Windows Overlay
- <img src="../images/HMI_minimieren.png" width="600"/>
Make sure you are logged in as "Administrator".
   
## How to install Additional Software?
- <img src="../images/Additional_Software.png" width="600"/>
You can select the software you want to install by using the correct .kop file. 


