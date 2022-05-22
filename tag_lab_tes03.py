# -*- coding: utf-8 -*-
"""
Created on Sat May 21 16:21:51 2022

@author: Gregory Kuhn
"""
# -*- coding: utf-8 -*-
"""
Created on Sun May 15 11:30:35 2022
Python Script to Measure Decawave Tag; and improve /
results with K-filter
@author: Gregk
"""

"""Importing modules and libraries """
import serial
import time
from datetime import datetime
import numpy as np
##import multiprocessing as mp

""" Below are the arrays that will be useful for Kalman Filter"""
A = np.array([[1,0],[0,1]]) # A matrix for converting state model
At= np.transpose(A)
B = np.array([[.5,0],[.5,0]],dtype=float) # B matrix for converting control matrix
W = np.array([[.05],[0.025]])# Predict State error matrix
Q = np.array([[.000212],[.04]]) #Error in the Predict State Matrix
R = np.array([[.05],[.05]]) #Measurment Uncertainty Matrix
Pc = np.array([[.05,0.0],[0.0,.05]])#Process Uncertainty Matrix
I = np.array([[1,0],[0,1]]) # Identity Matrix
H = np.array([[1,0],[0,1]]) ##Kalman Gain Conversion Matrix
C = np.array([[1,0],[0,1]]) ##Measurement to Observation matrix

"""Code below is used to establish a serial connection between listner """
port = "COM10"
baudrate = 115200
ser = serial.Serial(port, baudrate, timeout=1)

if ser.isOpen():
    print("Connected to " + ser.name)
    time.sleep(1)
    line = ser.write("\r\r".encode())
    time.sleep(1)
  
""" the function below is used  to print the position of the tag nodes"""
def print_tag_loc():
    ser.write('lec\r'.encode())
    ser.write('av\r'.encode())
    tag_loc = ser.readline()
    return(tag_loc)
    
""" The function below splits the tag location """
def split_tag_loc(tag_loc):
    if len(tag_loc) >=30:
        tag_pos = tag_loc.decode('utf-8')
        tag_pos = tag_pos.split(',')
        return(tag_pos)
    else:
        return None

def parse_tag_loc(tag_pos):
    tag_data  =  []
    tag_name  =  []
    tag_x_loc =  []
    tag_y_loc =  []
    tag_qf    =  []
    if tag_pos == None:
        return None
    else:    
        for place,item in enumerate(tag_pos):
            if item.find('POS') != -1:
                tag_name.append(tag_pos[place+2])
                tag_x_loc.append(tag_pos[place+3])
                tag_y_loc.append(tag_pos[place+4])
                tag_qf.append(tag_pos[place+6])
                z = [tag_name, tag_x_loc, tag_y_loc, tag_qf]
                tag_data.append(z)
                return(tag_data)

if __name__ == "__main__":
    while (1):
        try:
            date = datetime.now().strftime("%H:%M:%S")
            tag_loc = print_tag_loc()
            tag_pos = split_tag_loc(tag_loc)
            tag_data = parse_tag_loc(tag_pos)
            if (tag_data != None):
                print(f"At time {date} the tag is at position {tag_data}")
            time.sleep(1)
        except KeyboardInterrupt:
            ser.close()
            
