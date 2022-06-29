# -*- coding: utf-8 -*-
"""
Created on Sun May 15 11:30:35 2022
Python Script to Measure Decawave Tag; and improve /
results with K-filters
@author: Gregk
"""

"""Importing modules and libraries """
import serial
import time
import datetime
import numpy as np
import multiprocessing as mp

""" Below are the arrays that will be used for Kalman Filtering"""
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
port = "COM4"
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
    line = ser.readline()
    return(line)

    if __name__ == "__main__":
        tag_postion = print_tag_loc()
        ###
