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
np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
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

""" Global variabels """
dT = 0
init  = False
count = 0

"""Code below is used to establish a serial connection between listner """
port = "COM10"
baudrate = 115200
ser = serial.Serial(port, baudrate, timeout=1)

if ser.isOpen():
    print("Connected to " + ser.name)
    time.sleep(1)
    line = ser.write("\r\r".encode())
    time.sleep(1)
""" 
The function below is used  to print the position of the tag nodes

Returns:
    tag_loc:The unparsed dataline returned from the command lec\r
"""
def print_tag_loc():
    ser.write('lec\r'.encode())
    tag_loc = ser.readline()
    return(tag_loc)   
""" 
The function below decodeds and splits the tag_loc into a comma delimited string
Args:
    tag_loc:The unparsed dataline returned from the command lec\r

Returns:
    tag_pos: A parsed and decoded string consiting of tags position,name, and quality factor
"""
def split_tag_loc(tag_loc):
    if len(tag_loc) >=30:
        tag_pos = tag_loc.decode('utf-8')
        tag_pos = tag_pos.split(',')
        return(tag_pos)
    else:
        return None    
""" The function below parses and returns the important tag location data 
Args: 
    tag_pos: A parsed and decoded string consiting of tags position,name, and quality factor

Returns:
    tag_name: The name of the tag
    tag_obs:  The observed x and y coordinates of the tag     
    tag_qf:   The quality factor 
"""
def parse_tag_loc(tag_pos):
    tag_name  =  []
    tag_x_loc =  []
    tag_y_loc =  []
    tag_qf    =  []  
    for place,item in enumerate(tag_pos):
        if item.find('POS') != -1:
                tag_name.append(tag_pos[place+2])
                tag_x_loc.append(tag_pos[place+3])
                tag_y_loc.append(tag_pos[place+4])
                tag_qf.append(tag_pos[place+6])
                tag_obs = [tag_pos[place+3], tag_pos[place+4]]
                return(tag_name,tag_obs,tag_qf)

""" 
Function to predict the state of the tag based on its previous state estimate and acceleration 
Args:
    X_est: The previous(initial if first time being called) esistamed state of the tag
    Accel: The X and Y acceleration of the 

Returns: 
    X_est:
    
"""
def predict_state(X_est, Accel):
    global dT
    B = np.array([[.5*(dT*dT),0],[0,.5*(dT*dT)]],dtype=float) # B matrix for converting control matrix matrix
    X_est = np.dot(A,X_est) + np.dot(B,Accel)
    return(X_est)

if __name__ == "__main__":
    current_time = time.time()
    while (1):
        try:
            date = datetime.now().strftime("%H:%M:%S")
            tag_loc = print_tag_loc()
            tag_pos = split_tag_loc(tag_loc)
            if (tag_pos != None):
                    tag_name,tag_obs,tag_qf = parse_tag_loc(tag_pos)
                    print(f"At time {date} the tag  {tag_name} is at position {tag_obs} with a quality factor of {tag_qf}")
                    current_time = time.time()
            time.sleep(1)
            dT = round((time.time() - current_time),3)
            print(dT)
        except KeyboardInterrupt:
            ser.close()
            
