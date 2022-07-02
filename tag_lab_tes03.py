
"""
Created on Sun Jun 26 16:59:03 2022

This file is used to test ability of  only one Decawave's tag's position to be
tracked and predicted with a kalman filter. Connect to the tag pi is a piHat
and one Decawave Tag

@author: gkuhn
"""

"""Importing modules and libraries """
import serial
import time
import datetime
import numpy as np
from sense_hat import SenseHat
sense = SenseHat()
np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

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


## Establish a serial coonection
baudrate = 115200
port1 = "/dev/ttyAMA0"
tag1 = serial.Serial(port1, baudrate, timeout = 1) ##tag_apg
time.sleep(1)

if ser.isOpen():
    print("Connected to " + ser.name)
    time.sleep(1)
    line = ser.write("\r\r".encode())
    time.sleep(1)

 #Returns acceleration of tag
def get_accel():
    Accel = sense.get_accelerometer_raw()
    X = accel['x']
    Y = accel['y']
    Accel_list = [X,Y]
    return(Accel_list)

## return the position of the tag node
def print_tag_loc():
    ser.write('apg\r'.encode())
    line = ser.readline()
    return(line)

## return the position of the tag node
def tag_decode(line):
    Line = line.split()
    Line = Line[1:]
    X_pos = round(float((Line[0].strip('x:'))) * 1e-3 + .05,4)
    Y_pos = round(float((Line[1].strip('y:'))) * 1e-3 + .05,4)

##Function to return the quality factor
def sort_qf(line):
    Line = line.split(" ")
    Line = Line[1:]
    Qf =    (Line[3].strip('qf:'))
    return(Qf)

## Function to predict the state of the tag based on its previous state estimate and acceleration
def predict_state(X_est, Accel):
    global dT
    B = np.array([[.5*(dT*dT),0],[0,.5*(dT*dT)]],dtype=float) # B matrix for converting control matrix matrix
    X_est = np.dot(A,X_est) + np.dot(B,Accel)
    return(X_est)

## Function to predict the state of the tag based on its previous state estimate and accelera
def predict_cov(Pc):
    global LOS
    if LOS == True:
        Pc = np.array([[(delta_X * delta_X),0.0], [0.0,delta_Y*delta_Y]], dtype=float)
        LOS = False
    else:
        Pc = np.dot(A,Pc)
        Pc = np.dot(Pc,At) + Q
        Pc[0][1] = 0.0
        Pc[1][0] = 0.0
    return(Pc)

## Function to calculate the Kalman Gain
def Kalman_Gain(X_est,Pc):
    global LOS
    if LOS == True:
        Kg = np.array([[.833,0.0],[0.0,.833]])
        LOS = False
    else:
        Kg_num = np.dot(Pc,H)
        Kg_den = np.dot(H,Pc)
        Kg_den = np.dot(Kg_den,H) + R
        Kg     = np.divide(Kg_num,Kg_den)
        Kg[0][1] = 0.0
        Kg[1][0] = 0.0
    return(Kg)

##Function to update the state estimate; taking in account predicted, measured states and acceleration
def update_state(X_est,tag_apg,Kg):
    num = tag_apg - np.dot(H,X_est)
    X_est = X_est + np.dot(Kg,num)
    print()
    return(X_est)

##Function to update the process covariance matrix
def update_PC(Pc,Kg):
    num = I- (np.dot(Kg,H))
    Pc = np.dot(num,Pc)
    Pc[0][1] = 0.0
    Pc[1][0] = 0.0
    return(Pc)

if __name__ == "__main__":
    while True:
        time_now = datetime.datetime.now().strftime("%H:%M:%S")
        accel = get_accel()
        tag_apg = print_tag_loc()
        tag_loc = tag_decode(tag_apg)
        if tag_loc is not empty:
            count +=1
        if count == 3:
            init = True
        if init == True:
            try:

            except KeyboardInterrupt:
            print("Error! keybord interrupt detected, now closing the ports")
            tag1.close()
        time.sleep(1)
