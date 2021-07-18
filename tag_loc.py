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
B = np.array([[.5,0],[.5,0]],dtype=float)
W = np.array([[.05],[.05]])
At= np.transpose(A)
q = np.array([[.001,.001],[.002,.002]])
H = np.array([[1,0],[0,1]])
R = np.array([[.05,0],[0,.05]])
C = np.array([[1,0],[0,1]])
I = np.array([[1,0],[0,1]])


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

def process_cov():
    Pc= np.array([[.05,.05],[.05,.05]])
    Pc = np.dot(A,Pc)
    Pc = np.dot(Pc,At)+ W
    #print('Process covariance is')
    return(Pc)

def KalmanGain(X_est,Pc):
    Kg_num = np.dot(Pc,H)
    Kg_den = np.dot(H,Pc)
    Kg_den = np.dot(Kg_den,H) + R
    Kg     = np.divide(Kg_num,Kg_den)
    return(Kg)


def Observation(parse):
    tag_loc = np.array([[parse[0].strip('x:')],[parse[1].strip('y:')]],dtype=float)
    obs = np.dot(C,tag_loc)
    return(obs)

def Current(X_est,Kg,obs):
    X_num = obs - np.dot(Y,X_est)
    X_current = X_est+np.dot(Kg,X_num)
    return(X_current)

def UpdateProcess(Pc,Kg):
    Pcu = I - np.dot(K,H)
    Pc = np.dot(Pcu,Pc)
    return(Pc)

while True:
   time_now = time.strftime("%H:%M:%S")
   tag_pos = get_pos()
   accel = get_accel()


   if tag_pos:
       print('At time {0} the tag is at location {1} and is acellerating at {2}m/s^2'.format(time_now,tag_pos,accel))
       loc = predict_state(tag_pos,accel)
       pc = process_cov()
       kg = KalmanGain(loc,pc)
       obs = Observation(tag_pos)


       print("\n")
   time.sleep(.5)
