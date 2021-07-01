# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import serial
import datetime
import time
from sense_hat import SenseHat
import numpy as np

init = False
con_sec = 0
time_s = 0
sense = SenseHat()


ser =  serial.Serial('/dev/ttyACM0',115200, timeout = 1)

if ser.isOpen:
    print("connected to " + ser.name)
    time.sleep(1)
    line = ser.write("\r\r".encode())
    time.sleep(1)
    ser.write("lec\r".encode())
    
def get_Accel():
    accel = sense.get_accelerometer_raw()
    X = accel['x']
    Y = accel['y']
    Z = accel['z']
    accel_list = [X,Y,Z]
    return(accel_list)

    
    

while True:
    line=ser.readline()
    #print(line)
    if(line):
        if len(line) > 140 and 'POS'.encode() in line:
            con_sec+= 1
            parse = line.decode().split(",")
            x_pos = parse[parse.index("POS")+1]
            y_pos = parse[parse.index("POS")+2]
            z_pos = parse[parse.index("POS")+3]
            qf = parse[parse.index("POS")+4]
            accel = get_Accel()
            accel_x = accel[0]
            accel_y  = accel[1]
            #if init == 1:
            if (con_sec >=3 or init):
                time_now = time.strftime("%H:%M:%S")
                print("At time " + time_now + "  The tag is at position {0} {1}" 
                      .format(x_pos,y_pos)  )
                print("And is accelerating at {0:.5f} m/s^2 in the X and {1:.5f} in the "
                      "Y" .format(accel_x,accel_y))
                time.sleep(1)
                init = True
        else:
            con_sec = 0
    #print("Consecutuve iterations {}" .format(con_sec))
            
             
