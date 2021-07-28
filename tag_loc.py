 # -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file. mmmmmm,
"""

import serial
import time
from sense_hat import SenseHat
import numpy as np
import random

sense = SenseHat()
A = np.array([[1,0],[0,1]]) # A matrix for converting state model
I = np.array([[1,0],[0,1]]) 
B = np.array([[.5,0],[.5,0]],dtype=float) # B matrix for converting control matrix
W = np.array([[.05],[-.025]])# Predict error matrix
At= np.transpose(A)
Q = np.array([[.001],[.002]])
H = np.array([[1,0],[0,1]])
R = np.array([[.05],[.05]])
C = np.array([[1,0],[0,1]])

init = False 

ser =  serial.Serial('/dev/ttyACM0',115200,timeout = 1)

if ser.isOpen:
    print("connected to " + ser.name)
    print("")
    time.sleep(1)
    line = ser.write("\r\r".encode())
    time.sleep(1)
    
    #Function to print measured value of position   
def print_pos():
    ser.write('apg\r'.encode())
    line = ser.readline()
    if 'dwm'.encode() not in line and len(line)>10:
        parse = line.decode().split()
        parse = parse[1:]
        return(parse)
    else:
        return()  
        
       #Function to get measured value of position(wuith measured error)
def get_pos(parse):
    X_pos = float(parse[0].strip('x:'))*1e-3
    Y_pos = float(parse[1].strip('y:'))*1e-3
    Tag_loc = np.array([[X_pos],[Y_pos]],dtype=float) 
    Tag_loc = Tag_loc + W
    print()
    print('tag measure')
    print(Tag_loc)
    print()
    return(Tag_loc)

 #Gets acceleration
def get_accel():
    accel = sense.get_accelerometer_raw()
    X = accel['x']
    Y = accel['y']
    Z = accel['z']
    Accel_list = [X,Y,Z]
    return(Accel_list)
    
    #Preidc the state 
def predict_state(Tag_loc,Accel_list):
    global init
    global temp
    accel = np.array([[Accel_list[0]],[Accel_list[1]]],dtype=float)
    if init == False:
        X_est = np.dot(A,Tag_loc) + np.dot(B,accel)+Q
        init = True
        temp = X_est
    elif init == True:
        X_est = np.dot(A,temp) + np.dot(B,accel)+Q
    
    temp = X_est
    print()
    print('State_Est')
    print(X_est)
    print()
    return(X_est)
    
def process_cov():
    Pc= np.array([[.05,0.0],[0.0,.05]])
    Pc = np.dot(A,Pc)
    Pc = np.dot(Pc,At)+ W
    print()
    return(Pc)

def KalmanGain(X_est,Pc):
    
    Kg_num = np.dot(Pc,H)
    Kg_den = np.dot(H,Pc)
    Kg_den = np.dot(Kg_den,H) + R
    Kg     = np.divide(Kg_num,Kg_den)
    Kg[0][1] = 0.0
    Kg[1][0] = 0.0
    print('Kalman Gain')
    print(Kg)
    print()
    return(Kg)


def update_state(X_est,Tag_loc,KG):
    num = Tag_loc - np.dot(H,X_est)
    X_est = X_est + np.dot(KG,num)
    print()
    print('Updated State')
    print(X_est)
    print()
    return(X_est)

def update_process(Pc,KG):
    num = I- (np.dot(KG,H))
    Pc = np.dot(num,Pc)
    Pc[0][1] = 0.0
    Pc[1][0] = 0.0
    print()
    return(Pc)

while True:
   time_now= time.strftime("%H:%M:%S")   
   tag_pos = print_pos()
   accel = get_accel()
   
   if tag_pos:
       print('At time {0} the tag is at location {1} and is acellerating at {2}m/s^2'.format(time_now,tag_pos,accel))
       tag_loc = get_pos(tag_pos)
       est = predict_state(tag_loc,accel)
       pc = process_cov()
       print(pc)
       kg = KalmanGain(est,pc)
       est = update_state(est,tag_loc,kg)
       pc  = update_process(pc,kg)
       
   time.sleep(.5)
