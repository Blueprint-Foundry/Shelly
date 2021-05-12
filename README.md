![Shelly](https://github.com/Blueprint-Foundry/Shelly/blob/main/Docs/shelly-logo-black.jpg)
Shelly aims to be a low cost ROS robot for teaching and R&amp;D for makers, universities and corporations.

[![Compile Sketch](https://github.com/Blueprint-Foundry/Shelly/actions/workflows/compile-test.yml/badge.svg)](https://github.com/Blueprint-Foundry/Shelly/actions/workflows/compile-test.yml)

## Links
* Circuitmaker page for the Hardware: https://workspace.circuitmaker.com/Projects/Details/Ilia-Baranov/Shelly

## Dependencies
These must be installed to your environment
* ESP32 Arduino Board Manager https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md
* Arduino Library https://github.com/bblanchon/ArduinoJson
* Arduino Library https://github.com/Links2004/arduinoWebSockets
* Arduino Library https://github.com/Blueprint-Foundry/Arduino-MAX17055_Driver
* Arduino Library https://github.com/Blueprint-Foundry/PI4IOE5V96248_Arduino_Library
* Arduino Library https://github.com/Blueprint-Foundry/EAAPMST3923A2_Arduino_Library
* Arduino Library https://github.com/pololu/vl53l1x-arduino.git
* Arduino Library https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary
(note: ensure to enable DMP functionality. Edit ICM_20948_C.h and uncomment line 29: #define ICM_20948_USE_DMP

## Licenses
* Hardware is licensed under Attribution-NonCommercial 4.0 International (CC BY-NC 4.0): https://creativecommons.org/licenses/by-nc/4.0/
* Software is licensed under BSD 3 Clause: https://opensource.org/licenses/BSD-3-Clause
* Documentation is licensed under Attribution-ShareAlike 2.0 Generic (CC BY-SA 2.0): https://creativecommons.org/licenses/by-sa/2.0/
* This project intends to submit for Open Source Hardware certification once ready, this certification mark will be added once successfully granted. https://certification.oshwa.org/
* All rights reserved by Blueprint Foundry. 
