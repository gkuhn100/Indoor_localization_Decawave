# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import serial
import datetime
import sys
import time
from sense_hat import SenseHat
import numpy as np


sense = SenseHat()


ser =  serial.Serial('/dev/ttyACM0',115200, timeout = 1)

if ser.isOpen:
    print("connected to " + ser.name)
    time.sleep(1)
    line = ser.write("\r\r".encode())
    time.sleep(1)
    
def get_pos():
    ser.write('apg\r'.encode())
    line = ser.readline()
    parse = line.decode().split()
    print(parse)
    
    
def get_accel():
    accel = sense.get_accelerometer_raw()
    X = accel['x']
    Y = accel['y']
    Z = accel['z']
    accel_list = [X,Y,Z]
    return(accel_list)

    
    

while True:
   time_now= time.strftime("%H:%M:%S")
   print(time_now)
   get_pos()
   accel = get_accel()
   time.sleep(1)
