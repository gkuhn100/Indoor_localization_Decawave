 # -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file. mmmmmm,
"""

import serial
import datetime
import sys
import time
from sense_hat import SenseHat
import numpy as np

sense = SenseHat()
A = np.array([[1,0],[0,1]])
B = np.array([[.5,0],[.05,0]],dtype=float)
W = np.array([[.05],[.05]])

ser =  serial.Serial('/dev/ttyACM0',115200, timeout = 1)

if ser.isOpen:
    print("connected to " + ser.name)
    time.sleep(1)
    line = ser.write("\r\r".encode())
    time.sleep(1)
    
def get_pos():
    ser.write('apg\r'.encode())
    line = ser.readline()
    if 'dwm'.encode() not in line and len(line)>10:
        parse = line.decode().split()
        x_pos = parse[parse.index("apg:")+1].strip("x:")
        y_pos = parse[parse.index("apg:")+2].strip("y:")
        qf   = parse[parse.index("apg:")+4].strip("qf:")
        tag_pos = np.array([x_pos,y_pos,qf])
        parse = parse[1:]
        return(parse)
    else:
        return()   
def get_accel():
    accel = sense.get_accelerometer_raw()
    X = accel['x']
    Y = accel['y']
    Z = accel['z']
    accel_list = [X,Y,Z]
    return(accel_list)
    
def predict_state(parse,accel_list):
    tag_loc = np.array([[parse[0].strip('x:')],[parse[1].strip('y:')]],dtype=float)
    accel = np.array([[accel_list[0]],[accel_list[1]]],dtype=float)
    X_est = np.dot(A,tag_loc) + np.dot(B,accel)
    print('And the state estimate is {0}'.format(X_est))
    return(X_est)
while True:
   time_now= time.strftime("%H:%M:%S")   
   tag_pos = get_pos()
   accel = get_accel()
   
   
   if tag_pos:
       print('At time {0} the tag is at location {1} and is acellerating at {2}m/s^2'.format(time_now,tag_pos,accel))
       loc = predict_state(tag_pos,accel)
       print("\n")
   time.sleep(.5)
