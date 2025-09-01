# FabKit-Firmware
* =====================================================================================
* Filename:    main.cpp
* Project:     FabKit
* Version:     FWer
* Author:      Ioan Alexandru Ivan (for CEST)
* Date:        September 1, 2025
* Description:
*              This firmware controls the operation of FabKit sensors. It handles
*              the calibration, acquisition and display of different signals e.g. temperature, pressure, light,
*              distance, voltage, current etc., including communication over UART, LED control, etc.
*
* Updates:
*   Version 1.0.6b:
*     - MQTT Client IP has been changed to FabLab ethernet port
*     - Sensor Data MQTT messages are no longer retained
*   Version 1.0.6a:
*     - You can switch to Serial Communication from Wi-Fi only by sending Frequency 0 through MQTT
*     - Max Buffer Size over MQTT has been increased to 512
*   Version 1.0.6:
*     - Removed Serial Number changing
*     - Wi-Fi connection is now optional
*     - Data can now be sent to FabLab MQTT Broker via Wi-Fi
*     - Data Frequency can now be changed remotely through MQTT
*   Version 1.0.5:
*     - Added Wi-Fi connectivity
*     - Serial Number, SSID and Password can now be changed during setup
*   Version 1.0.4:
*     - Constant gains added to fix ADS1015 10% error
*   Version 1.0.3:
*     - FabCurrent code modification for complance with board ver 1.5 (still gets 10% error to be adjusted manually)
*     - Added Neopixel LED control
*     - EraseNVS command to factory-erase stored strings
*   Version 1.0.2:
*     - Added FabMove position/speed/acceleration and noise filtering
*     - Various bug fixes
*   Version 1.0.1:
*     - Added FabCurrent support
*     - Added ArduinoTask
*     - Increased speed for FabTemp
*   Version 1.0.0:
*     - First working Version with support for FabLight, FabTemp, FabPres
*
*
* Copyright (c) 2025 Ioan Alexandru Ivan
* All Rights Reserved.
*
* This software is proprietary to Ioan Alexandru Ivan and CEST. Unauthorized copying,
* redistribution, modification, or use of any part of this software is strictly prohibited
* without permission.
*
* For license inquiries, please contact  ioan.alexandru.ivan@gmail.com .
* =====================================================================================