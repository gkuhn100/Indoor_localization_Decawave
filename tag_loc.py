 # -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file. mmmmmm,
"""

import serial
import time
from sense_hat import SenseHat
import numpy as np

sense = SenseHat()
A = np.array([[1,0],[0,1]]) # A matrix for converting state model
B = np.array([[.5,0],[.5,0]],dtype=float) # B matrix for converting control matrix
W = np.array([[.0005],[-.025]])# Predict error matrix
At= np.transpose(A)
Q = np.array([[.001,.001],[.002,.002]])
H = np.array([[1,0],[0,1]])
R = np.array([[.05,0],[0,.05]])
C = np.array([[1,0],[0,1]])
Pc = np.array([[.05,0.0],[0,.05]])

init = False 

ser =  serial.Serial('/dev/ttyACM0',115200, timeout = 1)

if ser.isOpen:
    print("connected to " + ser.name)
    print("")
    time.sleep(1)
    line = ser.write("\r\r".encode())
    time.sleep(1)
    
def print_pos():
    ser.write('apg\r'.encode())
    line = ser.readline()
    if 'dwm'.encode() not in line and len(line)>10:
        parse = line.decode().split()
        parse = parse[1:]
        return(parse)
    else:
        return()  
        
def get_pos(parse):
    X_pos = float(parse[0].strip('x:'))*1e-3
    Y_pos = float(parse[1].strip('y:'))*1e-3
    tag_loc = np.array([[X_pos],[Y_pos]],dtype=float)
    
    return(tag_loc)

        
def get_accel():
    accel = sense.get_accelerometer_raw()
    X = accel['x']
    Y = accel['y']
    Z = accel['z']
    accel_list = [X,Y,Z]
    
    return(accel_list)
    
def predict_state(tag_loc,accel_list):
    global init
    global temp
    accel = np.array([[accel_list[0]],[accel_list[1]]],dtype=float)
    if init == False:
        print(init)
        X_est = np.dot(A,tag_loc) + np.dot(B,accel)+W
        temp = X_est
        init = True
    elif init == True:
        X_est = np.dot(A,temp) + np.dot(B,accel)+W
        temp = X_est
    
    return(X_est)
    
def process_cov():
    global Pc
    Pc = np.dot(A,Pc)
    Pc = np.dot(Pc,At)+ Q
    return(Pc)

def KalmanGain(X_est,Pc):
    
    Kg_num = np.dot(Pc,H)
    Kg_den = np.dot(H,Pc)
    Kg_den = np.dot(Kg_den,H) + R
    Kg     = np.divide(Kg_num,Kg_den)
    return(Kg)


def Observation(tag_pos):  
    obs = np.dot(C,tag_pos)
    return(obs)
    
while True:
   time_now= time.strftime("%H:%M:%S")   
   tag_pos = print_pos()
   accel = get_accel()
   
   
   if tag_pos:
       print('At time {0} the tag is at location {1} and is acellerating at {2}m/s^2'.format(time_now,tag_pos,accel))
       tag_loc = get_pos(tag_pos)
       Est = predict_state(tag_loc,accel)
       pc = process_cov()
       kg = KalmanGain(Est,pc)
       obs = Observation(tag_loc)
       
       print("\n")
   time.sleep(.5)
