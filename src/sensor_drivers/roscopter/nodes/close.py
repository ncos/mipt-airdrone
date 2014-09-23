#!/usr/bin/env python
import serial
serdev = '/dev/ttyACM0'
s  = serial.Serial(serdev)
s.close()
