# BLE and I2S demo
Simple demo application showing how to work with BLE service, BMA280 (Digital triaxial acceleration sensor) and digital I2S serial bus.

About this project
==================
Has one BLE service with *two characteristics*:
1. Audio (write only), **1 byte**
- **0x01** - *Start to play the first melody*
- **0x02** - *Start to play the second melody*
- **0x03** - *Stop playing*

2. Accelerometer (read only), **6 bytes**.

Contains the last accelerometer's values, written in *unsigned Big Endian format*, **2 bytes for each value (x,y,z)**.

Hardware requirements
---------------------
- **nRF52840-DK (PCA10056)**
- **BMA280 acceleration sensor**
- **Any I2S amplifier (tested on MAX98357A)**

Software requirements
---------------------
- *nRF5 SDK v17.1.0 (Tested)*
- *Latest version of Segger Embedded Studio*
- *nRF Connect or LightBlue mobile application*

The project may need modifications to work with other versions or other boards.

How to get started
------------------
1. Create new directory (**"Projects"**, for example).
2. Place SDK folder to **"Projects"** folder and rename it to **"nRF5_SDK_Current"**.
3. Clone the repository to this folder too. Directory structure should now look like this: 
- **/Projects**
- **/Projects/nRF5_SDK_Current**
- **/Projects/ble_and_i2s_demo**
4. Now you can connect your modules to nRF Board:

**I2S module pins:**
- SCK PIN P0.31
- LRCK PIN P0.29
- SDOUT PIN P0.30

**BMA280 module pins:**
- INT2 PIN P1.13
- SCL PIN  P0.27
- SDA PIN  P0.26

5. Now you can open project file "**ble_and_i2s_app_demo/pca10056/s140/ses/ble_and_i2s_app_demo_pca10056_s140.emProject**" in *Segger Embedded Studio*, compile, flash and run application.