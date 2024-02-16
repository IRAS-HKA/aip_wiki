# Trouble shooting 

This section provides a brief summary of various problems that have arisen as well as their solution.  This may help you with different kind of struggles while working with the AIP application. 


## I.  PLC Device Manager not accessible

`The plc device manager is not accessible`

### Solution:
- Reset PLC to factory settings
- PLC settings are available via "CER Host". Program is available in MS Teams folder.


## II. EL 1904 Diag 2 (red) illuminating

`Error: The Diag 2 LED illuminates red if the terminal detects an external supply or cross-circuit. The LED
extinguishes once the error is rectified.`

### Solution:
- Check circuit wiring
- Check if correct safety configuration is loaded to PLC
- Check if correct device description file (ESI) for KRC4 EL6995-1001 is loaded

## III. PLC does not provide the necessary safety approval to KUKA KR C4

`Possible errors:
Programm is not loaded correctly to PLC,
SD card is broken`

### Solution:

- If the program is not loaded correctly, it is not possible to switch the PLC into "RUN" mode. Try to load the program again. The complete project is saved in the  it is saved in corresponding AIP MS Teams Team: : "CER Host"
- If the SD card is broken, simply copy the files to a new one. No flashing required.

## IV. PLC debugging issues

`Various plc debugging issues`

- Connect PLC via display output to a monitor
- Install programm "**CER Host**" from the corresponding AIP MS Teams Team _IRAS Students/Projects/.-Automated_Item_Picking_ and access it over this tool. 
- Access it over the web interface -> PW required
- If nothing works: Take SD card to another PLC unit or try it there.

## V. Network communication issues

`Devices can´t establish communication in the network`

### Solution:

- Check the cables and cable connections
- Verify that all devices are in the same subnet (e.g. 10.177.x.x) 
- Log onto device to check if they are running/working (connection via ethernet/usb to pc)

## VI. IO-Link and KUKA KR C4 connection issues

`IO-Link and KUKA KR C4 connection issues`

### Solution:
- Install KUKA WorkVisual and IFM Moneo and check the current configuration
- IO Link Master IFM 24001 can be resetted
- Check if devices are listed in the graphical overview in Workvisual. If not,import them and deploy the project to KUKA.

## VII. PLC License Error

`Invalid license for PLC`

If the battery of the PLC is empty, the date will be resetted after every restart to a date in the past. This triggers license errors preventing you to load the configuration to the physical modules. Free 7 days licenses are generated based on the current date and time of your PC.

### Solution

- Long term: Replace the button battery of the PLC
- Short term: Set the correct date and time manually in the online device manager. This is required at every restart of the system.
