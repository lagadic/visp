#!/usr/bin/env python

import time
import serial

ser = serial.Serial(
  port='/dev/ttyAMA0',
  baudrate = 9600,
)

x = ser.write('MOTOR_RPM=-100,100\n')
time.sleep(0.5)
x = ser.write('MOTOR_RPM=-50,100\n')
time.sleep(0.5)
x = ser.write('MOTOR_RPM=50,-50\n')
time.sleep(0.5)
x = ser.write('LED_RING=0,0,10,0\n')
time.sleep(0.5)
x = ser.write('LED_RING=0,0,0,10\n')
time.sleep(0.5)
x = ser.write('LED_RING=0,0,0,0\n')
time.sleep(0.5)
ser.close()

