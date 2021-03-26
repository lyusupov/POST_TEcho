# POST T-Echo

Power-on Self Test for LilyGO&#174; **T-Echo** (Nordic nRF52840 based) LoRaWAN GNSS tracker

* [Illustrations](https://github.com/lyusupov/POST_TEcho/blob/main/README.md#illustrations)
* [Disclaimer](https://github.com/lyusupov/POST_TEcho/blob/main/README.md#disclaimer)
* [Installation](https://github.com/lyusupov/POST_TEcho/blob/main/README.md#installation)
* [Credits](https://github.com/lyusupov/POST_TEcho/blob/main/README.md#credits)

# Illustrations

<br>

<img src="https://github.com/lyusupov/POST_TEcho/blob/main/docs/images/POST_T-Echo_1.jpg" height="313" width="400">

<br>

# Disclaimer

This software is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

# Installation

The **T-Echo** typically comes with factory pre-installed [Adafruit_nRF52_Bootloader](https://github.com/adafruit/Adafruit_nRF52_Bootloader).<br>
The Bootloader is capable to self-program an application firmware into the device. In order to simplify the firmware transfer, the bootloader emulates a "USB Mass Storage" interface.

1. Download an appropriate variant of the CircuitPython firmware binary from [this location](https://github.com/lyusupov/POST_TEcho/tree/main/bin) ;

2. Connect the LilyGO&#174; **T-Echo** to your PC by means of a USB cable (Type-A <-> Type-C) ;

3. Double click (within 0.5 seconds) onto the device RESET button. A virtual disk with **NRF52BOOT** label should appear in your "File manager" afterwards.

4. Drag the downloaded firmware file by your pointing device (mouse, trackball,...) , then drop it into NRF52BOOT disk. Wait until the file transfer is complete.

<br>

![](https://github.com/lyusupov/POST_TEcho/blob/main/docs/images/POST_T-Echo_3.jpg)

<br>

5. Copy files from _**src**_ folder into the "USB Mass Storage" as follows: 

<br>

![](https://github.com/lyusupov/POST_TEcho/blob/main/docs/images/POST_T-Echo_2.jpg)

<br>


# Credits

Name|Subject
---|---
[LilyGO company](http://www.lilygo.cn/)|**TTGO T-Echo**
[Adafruit Industries](http://adafruit.com/)|[CircuitPython](https://github.com/adafruit/circuitpython) , [e-Paper](https://github.com/adafruit/Adafruit_CircuitPython_EPD) and [BME280](https://github.com/adafruit/Adafruit_CircuitPython_BME280) libraries 
[Lewis He](https://github.com/lewisxhe)|[MicorPython PCF8563 library](https://github.com/lewisxhe/PCF8563_PythonLibrary)
[Ehong-tl](https://github.com/ehong-tl)|[Port of SX1262 driver](https://github.com/ehong-tl/micropySX126X) from [RadioLib](https://github.com/jgromes/RadioLib) library for MicroPython
