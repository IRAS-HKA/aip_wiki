# How to configure Sick Scanner MicroScan3 Core I/O
## Hardware

The scanner operates on a 24V power supply, while its signals, known as OSSDA and OSSDB, are redundantly connected to the PLC. These signals interface with two distinct channels of the PLC module EL1904. The scanner is equipped with additional universal inputs and outputs. These can be optionally configured.

## Software

The scanner is configured with the configuration tool "[Safety Designer](https://www.sick.com/de/en/catalog/products/safety/safety-controllers/safety-designer/c/g575306?tab=overview)". In the following the most important steps of Safety Designer are explained. The tool is very intuitive.

- After the software is installed, open it and connect your computer via USB with the scanner. Search for the connected scanner.
- Choose the scanner to open the **configuration**.

    <img src="../images/20240213_Choose_Scanner_SD.png" width="300"><br>

- In the tab "**Configuration**" the settings of the scanner can be adjusted.

    <img src="../images/20240213_Configuration_Scanner_SD.png" width="500"><br>

- Choose "**Fields**" to configure the warning and stop field of the scanner.

    <img src="../images/20240213_Fields_Scanner_SD.png" width="500"><br>

- Choose "**In- and Outputs, local**" to assign a funtion to a pin of the scanner.

    <img src="../images/20240213_InOuts_Scanner_SD.png" width="500"><br>

- Choose "**Monitoring cases**" to create or adapt a monitoring case. Pins can be assigned to the stop or warning field.

    <img src="../images/20240213_Monitoring_cases_SD.png" width="500"><br>

- Save your changes and load it to the scanner.
