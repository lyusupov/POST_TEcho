#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  LilyGO T-Echo Power-On Self Test
#
#  post.py (main.py)
#
#  Copyright (C) 2021 Linar Yusupov
# 
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
# 
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# 
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.

import digitalio
import busio
import board
import time
import sys

SOC_GPIO_PIN_IO_PWR   = board.P0_12

SOC_GPIO_LED_GREEN    = board.P0_15
SOC_GPIO_LED_RED      = board.P0_13
SOC_GPIO_LED_BLUE     = board.P0_14

SOC_GPIO_PIN_SS       = board.P0_24
SOC_GPIO_PIN_RST      = board.P0_25
SOC_GPIO_PIN_BUSY     = board.P0_17
SOC_GPIO_PIN_DIO1     = board.P0_20

SOC_GPIO_PIN_SWSER_RX = board.P1_09
SOC_GPIO_PIN_SWSER_TX = board.P1_08

SOC_GPIO_PIN_EPD_MISO = board.P1_07
SOC_GPIO_PIN_EPD_MOSI = board.P0_29
SOC_GPIO_PIN_EPD_SCK  = board.P0_31
SOC_GPIO_PIN_EPD_SS   = board.P0_30
SOC_GPIO_PIN_EPD_DC   = board.P0_28
SOC_GPIO_PIN_EPD_RST  = board.P0_02
SOC_GPIO_PIN_EPD_BUSY = board.P0_03
SOC_GPIO_PIN_EPD_BLGT = board.P1_11

SOC_GPIO_PIN_SDA      = board.P0_26
SOC_GPIO_PIN_SCL      = board.P0_27

SOC_GPIO_PIN_BUTTON   = board.P1_10

SOC_GPIO_PIN_SFL_MOSI = board.P1_12
SOC_GPIO_PIN_SFL_MISO = board.P1_13
SOC_GPIO_PIN_SFL_SCK  = board.P1_14
SOC_GPIO_PIN_SFL_SS   = board.P1_15
SOC_GPIO_PIN_SFL_HOLD = board.P0_05
SOC_GPIO_PIN_SFL_WP   = board.P0_07

has_radio  = False
has_gnss   = False
has_epaper = True
has_rtc    = False
has_baro   = False

from os import statvfs
fstat = statvfs('/')
has_flash = True if fstat[0] == 512 and fstat[2] > 2048 else False 

def io_power(val):
  io_pwr = digitalio.DigitalInOut(SOC_GPIO_PIN_IO_PWR)
  io_pwr.direction = digitalio.Direction.OUTPUT
  io_pwr.value = val

CMD_JEDEC_ID          = const(0x9F)

# MX25R1635F SPI flash
MACRONIX_ID           = const(0xC2)
MEMORY_TYPE           = const(0x28)
CAPACITY              = const(0x15)

def probe_flash():
  hold = digitalio.DigitalInOut(SOC_GPIO_PIN_SFL_HOLD)
  hold.switch_to_output(value=True)
  wp = digitalio.DigitalInOut(SOC_GPIO_PIN_SFL_WP)
  wp.switch_to_output(value=True)
  spi = busio.SPI(SOC_GPIO_PIN_SFL_SCK, MOSI=SOC_GPIO_PIN_SFL_MOSI, MISO=SOC_GPIO_PIN_SFL_MISO)
  cs = digitalio.DigitalInOut(SOC_GPIO_PIN_SFL_SS)
  cs.switch_to_output(value=True)
  while not spi.try_lock():
      pass
  spi.configure(baudrate=2000000, phase=0, polarity=0, bits=8)
  cs.value = False
  spi.write(bytes([CMD_JEDEC_ID]))
  in_ = bytearray(3)
  spi.readinto(in_)
  cs.value = True
  spi.unlock()
  spi.deinit()

  return True if in_[0] == MACRONIX_ID and in_[1] == MEMORY_TYPE and in_[2] == CAPACITY else False

def rgb_test():
  led_r = digitalio.DigitalInOut(SOC_GPIO_LED_RED)
  led_r.direction = digitalio.Direction.OUTPUT
  led_r.value = True
  led_g = digitalio.DigitalInOut(SOC_GPIO_LED_GREEN)
  led_g.direction = digitalio.Direction.OUTPUT
  led_g.value = True
  led_b = digitalio.DigitalInOut(SOC_GPIO_LED_BLUE)
  led_b.direction = digitalio.Direction.OUTPUT
  led_b.value = True

  from time import sleep
  for ndx in range(3):
    led_r.value = False
    sleep(0.5)
    led_g.value = False
    led_r.value = True
    sleep(0.5)
    led_b.value = False
    led_g.value = True
    sleep(0.5)
    led_b.value = True

def probe_sx1262():
  sys.path.append( '/lib/micropySX126X' )
  from sx1262 import SX1262
  sx = SX1262(cs=SOC_GPIO_PIN_SS, irq=SOC_GPIO_PIN_DIO1, rst=SOC_GPIO_PIN_RST,\
    gpio=SOC_GPIO_PIN_BUSY,clk=board.SCK, mosi=board.MOSI, miso=board.MISO)
  sx.reset()
  data = bytearray(1)
  data_mv = memoryview(data)
  SX126X_REG_LORA_SYNC_WORD_LSB = const(0x0741)
  SX126X_DEF_LORASYNCWORDLSB = const(0x24)
  state = sx.readRegister(SX126X_REG_LORA_SYNC_WORD_LSB, data_mv, 1)
  sx.sleep()
  return True if state == 0 and data[0] == SX126X_DEF_LORASYNCWORDLSB else False

def probe_sx1276():
  import adafruit_rfm9x
  RADIO_FREQ_MHZ = 868.0
  CS = digitalio.DigitalInOut(board.P0_24)
  RESET = digitalio.DigitalInOut(board.P0_25)
  RH_RF95_REG_42_VERSION = 0x42
  radio_spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
  rfm9x = adafruit_rfm9x.RFM9x(radio_spi, CS, RESET, RADIO_FREQ_MHZ)
  regver_val = rfm9x._read_u8(RH_RF95_REG_42_VERSION)
  #print("REG_version = ", regver_val)
  rfm9x.sleep()
  radio_spi.deinit()
  return True if regver_val == 0x12 else False

def probe_gnss():
  #from adafruit_gps import GPS
  uart_gps = busio.UART(SOC_GPIO_PIN_SWSER_TX, SOC_GPIO_PIN_SWSER_RX, baudrate=9600, timeout=30)
  #gps      = GPS(uart_gps, debug=False)

  time_marker = time.monotonic()

  rval = False
  while time.monotonic() - time_marker < 3.0:
    #gps.update()
    sentence = uart_gps.readline()
    prefix = sentence[0:2].decode("ascii")
    if prefix == '$G' or prefix == '$B':
      rval = True
      break
  return rval

def probe_rtc():
  import pcf8563
  i2c = busio.I2C(SOC_GPIO_PIN_SCL, SOC_GPIO_PIN_SDA)
  rtc = pcf8563.PCF8563(i2c)

  time_marker = time.monotonic()
  prev_seconds = rtc.seconds()
  rval = False
  while time.monotonic() - time_marker < 3.0:
    if rtc.seconds() != prev_seconds:
      rval = True
      break
  i2c.deinit()
  return rval

def probe_baro():
  import adafruit_bme280
  i2c = busio.I2C(SOC_GPIO_PIN_SCL, SOC_GPIO_PIN_SDA)
  bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)

  rval = False

  if bme280.pressure > 0:
    rval = True
  i2c.deinit()
  return rval

def epd_init():
  from adafruit_epd.epd import Adafruit_EPD
  from adafruit_epd.ssd1681 import Adafruit_SSD1681

  epd_spi  = busio.SPI(SOC_GPIO_PIN_EPD_SCK, MOSI=SOC_GPIO_PIN_EPD_MOSI,\
                                             MISO=SOC_GPIO_PIN_EPD_MISO)
  ecs      = digitalio.DigitalInOut(SOC_GPIO_PIN_EPD_SS)
  dc       = digitalio.DigitalInOut(SOC_GPIO_PIN_EPD_DC)
  rst      = digitalio.DigitalInOut(SOC_GPIO_PIN_EPD_RST)
  busy     = digitalio.DigitalInOut(SOC_GPIO_PIN_EPD_BUSY)
  srcs     = None

  display = Adafruit_SSD1681(200, 200, epd_spi, cs_pin=ecs, dc_pin=dc,\
    sramcs_pin=srcs, rst_pin=rst, busy_pin=busy)

  display.rotation = 1
  display.fill(Adafruit_EPD.WHITE)
  x = 30
  y = 40
  display.text('SoftRF', x, y, Adafruit_EPD.BLACK, size=4)
  x = 85
  y = 85
  display.text('and', x, y, Adafruit_EPD.BLACK, size=2)
  x = 30
  y = 120
  display.text('LilyGO', x, y, Adafruit_EPD.BLACK, size=4)
  display.display()
  return display

def display_status(display):
  from adafruit_epd.epd import Adafruit_EPD
  display.fill(Adafruit_EPD.WHITE)
  x = 10
  y = 2
  display.text('RADIO    ' + ('+' if has_radio  else '-'), x, y,\
                Adafruit_EPD.BLACK, font_name="font5x8.bin", size=3)
  y = y + 35
  display.text('GNSS     ' + ('+' if has_gnss   else '-'), x, y,\
                Adafruit_EPD.BLACK, size=3)
  y = y + 35
  display.text('DISPLAY  ' + ('+' if has_epaper else '-'), x, y,\
                Adafruit_EPD.BLACK, size=3)
  y = y + 35
  display.text('RTC      ' + ('+' if has_rtc    else '-'), x, y,\
                Adafruit_EPD.BLACK, size=3)
  y = y + 35
  display.text('FLASH    ' + ('+' if has_flash  else '-'), x, y,\
                Adafruit_EPD.BLACK, size=3)
  y = y + 35
  display.text('BARO     ' + ('+' if has_baro   else '-'), x, y,\
                Adafruit_EPD.BLACK, size=3)
  display.display()

io_power(True)
bl = digitalio.DigitalInOut(SOC_GPIO_PIN_EPD_BLGT)
bl.direction = digitalio.Direction.OUTPUT
bl.value = False

has_flash = True if has_flash else probe_flash()
has_radio = probe_sx1262()
#has_radio = probe_sx1276()
epd       = epd_init()
has_gnss  = probe_gnss()
has_rtc   = probe_rtc()
has_baro  = probe_baro()

print()
print('LilyGO T-Echo Power-on Self Test')
print()

print('Built-in components:')
print('RADIO   :', 'PASS' if has_radio else 'FAIL')
print('GNSS    :', 'PASS' if has_gnss  else 'FAIL')
print('RTC     :', 'PASS' if has_rtc   else 'FAIL')
print('FLASH   :', 'PASS' if has_flash else 'FAIL')
print('BARO    :', 'PASS' if has_baro  else 'FAIL')

print()
print('Power-on Self Test is completed.')
print()

display_status(epd)
bl.value = True
rgb_test()
bl.value = False

#import alarm
#pin_alarm = alarm.pin.PinAlarm(pin=SOC_GPIO_PIN_BUTTON, value=False, pull=True)
#alarm.exit_and_deep_sleep_until_alarms(pin_alarm)
