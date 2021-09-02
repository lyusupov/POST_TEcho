# POST T-Echo

Power-on Self Test for LilyGO&#174; **T-Echo** (Nordic nRF52840 based) LoRaWAN GNSS tracker

* [Illustrations](https://github.com/lyusupov/POST_TEcho/blob/main/README.md#illustrations)
* [Disclaimer](https://github.com/lyusupov/POST_TEcho/blob/main/README.md#disclaimer)
* [CircuitPython](https://github.com/lyusupov/POST_TEcho/blob/main/README.md#circuitpython)
* [Installation](https://github.com/lyusupov/POST_TEcho/blob/main/README.md#installation)
* [Credits](https://github.com/lyusupov/POST_TEcho/blob/main/README.md#credits)

**ATTENTION**: This test is designed to run on top of Adafruit **CiruitPython** interpreter.<br>
For an **Arduino** or **Platformio** IDE, please, use [this sketch](https://github.com/lewisxhe/nRF52840_UnitTest) created by [**Lewis He**](https://github.com/lewisxhe).

# Illustrations

## e-Paper display output

<br>

<img src="https://github.com/lyusupov/POST_TEcho/blob/main/docs/images/POST_T-Echo_1.jpg" height="313" width="400">

<br>

## REPL console output

<br>

![](https://github.com/lyusupov/POST_TEcho/blob/main/docs/images/POST_T-Echo_4.jpg)

<br>

# Disclaimer

This software is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

# CircuitPython

This power-on self test is running on top of [CircuitPython 6](https://github.com/adafruit/circuitpython) software from Adafruit Industies.

<br>

![](https://s3.amazonaws.com/adafruit-circuit-python/CircuitPython_Repo_header_logo.png)

<br>

These 'frozen' modules are integrated into the custom CircuitPython build in order to better match the LilyGO&#174; **T-Echo** board hardware specs:

* [Adafruit_CircuitPython_EPD](https://github.com/adafruit/Adafruit_CircuitPython_EPD)
* [Adafruit_CircuitPython_GPS](https://github.com/adafruit/Adafruit_CircuitPython_GPS)
* [Adafruit_CircuitPython_BME280](https://github.com/adafruit/Adafruit_CircuitPython_BME280)
* [Adafruit_CircuitPython_framebuf](https://github.com/adafruit/Adafruit_CircuitPython_framebuf)
* [Adafruit_CircuitPython_datetime](https://github.com/adafruit/Adafruit_CircuitPython_datetime)

# Installation

The **T-Echo** typically comes with factory pre-installed [Adafruit_nRF52_Bootloader](https://github.com/adafruit/Adafruit_nRF52_Bootloader).<br>
The Bootloader is capable to self-program an application firmware into the device. In order to simplify the firmware transfer, the bootloader emulates a "USB Mass Storage" interface.

1. Download integrated _(CircuitPython6 and scripts)_ **POST** firmware binary from [this location](https://github.com/lyusupov/POST_TEcho/tree/main/bin) ;

2. Connect the LilyGO&#174; **T-Echo** to your PC by means of a USB cable (Type-A <-> Type-C) ;

3. Double click (within 0.5 seconds) onto the device RESET button. A virtual disk with **NRF52BOOT** label should appear in your "File manager" afterwards.

4. Drag the downloaded firmware file by your pointing device (mouse, trackball,...) , then drop it into NRF52BOOT disk. Wait until the file transfer is complete.

<br>

![](https://github.com/lyusupov/POST_TEcho/blob/main/docs/images/POST_T-Echo_3.jpg)

<br>

# Credits

Name|Subject
---|---
[LilyGO company](http://www.lilygo.cn/)|[**TTGO T-Echo**](https://s.click.aliexpress.com/e/_9wz3pJ)
[Adafruit Industries](http://adafruit.com/)|[CircuitPython](https://github.com/adafruit/circuitpython) , [e-Paper](https://github.com/adafruit/Adafruit_CircuitPython_EPD) and [BME280](https://github.com/adafruit/Adafruit_CircuitPython_BME280) libraries 
[Lewis He](https://github.com/lewisxhe)|[MicroPython PCF8563 library](https://github.com/lewisxhe/PCF8563_PythonLibrary)
[Ehong-tl](https://github.com/ehong-tl)|[Port of SX1262 driver](https://github.com/ehong-tl/micropySX126X) from [RadioLib](https://github.com/jgromes/RadioLib) library for MicroPython
