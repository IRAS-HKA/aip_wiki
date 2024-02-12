# PLC Device Manager not accessible

### Solution
- Reset PLC to factory settings
- PLC settings are available via "CER Host". Program is available in MS Teams folder.


# EL 1904 Diag 2 (red) illuminating

`Error: The Diag 2 LED illuminates red if the terminal detects an external supply or cross-circuit. The LED
extinguishes once the error is rectified.`

### Solution
- Check circuit wiring
- Check if correct safety configuration is loaded to PLC
- Check if correct device description file (ESI) for KRC4 EL6995-1001 is loaded


# PLC does not give permission (= keine Freigabesignal der Sicherheitssteuerung für Roboter)
`Possible errors:
BIOS Battery is low,
Programm is not loaded correctly to PLC,
SD card is broken`

### Solution
- If the BIOS Battery is low, the time on PLC is set to 01.01.1970. This leads to the pseudo error of an expired license. Change battery on PLC below the SD Card reader and set time to actual time.
- If program is not loaded correctly, it is not possible to switch the PLC into "RUN" mode. Load the program again, it is saved in MS Teams
- If the SD card is broken, copy file simply to a new one. No flashing required


# SPS debugging problems
- Connect PLC via Display output to a monitor
- Install programm from MS Teams: "CER Host"
- Access it over the web interface -> PW required
- If nothing works: Take SD card to another SPS unit and try it there


# Devices can not find each other in the network

### Solution
- Check cable and connection of cable
- Check if all devices are in the same subnet (e.g. 10.177.x.x) 
- Log onto device to check if they are running/working (connection via ethernet/usb to pc)


# IO-Link and KUKA KR10 connection issues

### Solution
- Install KUKA Workvisual and IFM Moneo and check the configuration
- IO Link Master IFM 24001 can be reset
- Check if devices are in the grafical overview in Workvisual. If not import them and deploy project to KUKA.