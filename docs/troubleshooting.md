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


# SPS does not give permission (= keine Freigabesignal der Sicherheitssteuerung für Roboter)
possible problems:
- BIOS Battery is low -> Time on SPS is set to 1.1.1970 -> Error
- Programm is not loaded correctly to SPS -> load it again, programm is stored in MS Teams

### Solution
- BIOS Battery is low -> Change battery on SPS below the SD Card reader
- SD card is broken -> copy file simply to a new one and it should work, no flashing required
- 

# SPS debugging problems
- Connect via Display output to a monitor
- Install programm from MS Teams: CERHost
- Access it over the web interface -> PW required
- if nothing works: shut down SPS, take SD card to another SPS unit and try it there

# Devices can not find eachother in the network

### Solution
- check cable and connection of cable
- check if all devices are in the same subnet (e.g. 10.177.x.x) 
- log onto device to check if they are running/working (connection via ethernet/usb to pc)
  

# IO-Link and KUKA KR10 connection issues
### Solution
- Install KUKA Workvisual and IFM Moneo and check the configuration
- it helped to reset the IO Link Master IFM 24001 
- Check if devices are in the grafical overview in Workvisual -> if not import them and deploy project to KUKA