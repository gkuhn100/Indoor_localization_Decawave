#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov  2 20:04:03 2021

@author: pi
"""

#libraries and modules to import
import serial
import time
import numpy as np
from sense_hat import SenseHat
sense = SenseHat()

# Matrices
A =  np.array([[1,0],[0,1]]) # A matrix for converting state model
At = np.transpose(A) ## A transpose
B =  np.array([[.5,0],[.5,0]]) ## Matrix for converting control variable
W = np.array([[.05],[0.025]]) #Predict State error matrix
Q = np.array([[.000212],[.04]]) #Error in the Predict State Matrix
R = np.array([[.05],[.05]]) #Measurment Uncertainty Matrix
Pc = np.array([[.05,0.0],[0.0,.05]])#Process Uncertainty Matrix
I = np.array([[1,0],[0,1]]) # Identity Matrix
H = np.array([[1,0],[0,1]]) ##Kalman Gain Conversion Matrix
C = np.array([[1,0],[0,1]]) ##Measurement to Observation matrix


ser = serial.Serial('/dev/ttyACM0',115200,timeout = 1)
print('Connected to ' + ser.name)
print()
ser.write("\r\r".encode())
time.sleep(1)


##Function to get the acceleration of the tag

def get_accel():
    accel = sense.get_accelerometer_raw()
    accel = sense.get_accelerometer_raw()
    X = accel['x']
    Y = accel['y']
    X = round(X,3)
    Y = round(Y,3)
    Accel_list = [X,Y]
    return (Accel_list)


if __name__ == '__main__':
    while(1):
        accel = get_accel()
        time.sleep(1)
