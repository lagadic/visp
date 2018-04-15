#!/usr/bin/env python

# This test has to run on a laptop connected throw a USB
# to serial adapter to a Raspberry Pi.

import serial

ser = serial.Serial(         
  port='/dev/ttyAMA0',
  baudrate = 9600,
  timeout=0
)

x = ser.write('hello')
ser.close()

