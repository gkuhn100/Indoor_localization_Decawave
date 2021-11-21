import serial
import time
from sense_hat import SenseHat
import numpy as np
sense = SenseHat()

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

init = 0
i = 0
ser =  serial.Serial('/dev/ttyACM0',115200,timeout = 1)

if ser.isOpen:
    print("connected to " + ser.name)
    print("")
    time.sleep(1)
    line = ser.write("\r\r".encode())
    time.sleep(1)
    
    
def print_anchor():
    print('There are four Anchors in this setup')
    print('AN0 is named 0E15 and is located at 0.00 0.00')
    print('AN1 is named 1516 and is located at 2.20 0.00')
    print('AN2 is named 41B1 and is located at 2.20 4.55')
    print('AN3 is named 089F and is located at 0.00 4.55')
    print()
    
def print_tag_warning():
    print()

def print_anchor_warning():
    print()

    #Function to print measured value of position
def print_pos():
    ser.write('apg\r'.encode())
    line = ser.readline()
    if 'dwm'.encode() not in line and len(line)>10:
        #parse = line.decode().split()
        #parse = parse[1:]
        parse = ['x:.487', 'y:4.162', 'z:0.87', 'qf:78']
        return(parse)
    else:
        return()

       #Function to get measured value of position(with measured error)
def get_pos(parse):
    X_pos = float(parse[0].strip('x:'))*1e-3+ .532
    Y_pos = float(parse[1].strip('y:'))*1e-3+ 4.218
    Tag_loc = np.array([[X_pos],[Y_pos]],dtype=float)
    print()
    print('Current Tag Observation')
    print(Tag_loc)
    print()
    return(Tag_loc)
    
def det_qf(parse):
    qf = float(parse[3].strip('qf:'))*1e-3
    return(qf)

 #Gets acceleration of the Tag
def get_accel():
    accel = sense.get_accelerometer_raw()
    X = accel['x']
    Y = accel['y']
    Accel_list = [X,Y]
    return(Accel_list)

def predict_state(X_est,Accel_list):
    Accel = np.array([[Accel_list[0]],[Accel_list[1]]],dtype=float)
    X_est = np.dot(A,est) + np.dot(B,Accel)
    return(X_est)

    #Matrix to Predict the Kalman Gain Uncertainty in process vs measured
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

def get_qf(parse):
     Qf = float(parse[0].strip('x:'))*1e-3
     print('quality factor is {0}\n'.format(Qf))
     return(Qf)

print_anchor()
while True:
   if (i < 5):
       time_now= time.strftime("%H:%M:%S")
       tag_pos = print_pos()
       accel = get_accel()
       qf = 78
       if tag_pos:
           print('At time {0} the tag is at location {1} with a quality factor of {2} and is acellerating at {3}m/s^2'.format(time_now,tag_pos,qf,accel))
           tag_loc = get_pos(tag_pos)
           if init == 0:
               est = get_pos(tag_pos)   
           elif init >= 1:
               est = predict_state(est,accel)
               print('The State Estimate is \n')
               print(est)
               print()
               Pc = np.dot(A,Pc)
               Pc = np.dot(Pc,At)+Q
               Pc[1][0] = 0.0
               Pc[0][1] = 0.0
               print('The Proces Covariance is ')
               print(Pc)
               kg = KalmanGain(est,Pc)
               est = update_state(est,tag_loc,kg)
               num = I- (np.dot(kg,H))
               Pc = np.dot(num,Pc)
               Pc[0][1] = 0.0
               Pc[1][0] = 0.0
               print('The updated PC is ')
               print(Pc)
               print()
           else:
            est = tag_loc
           init += 1
       i+=1
       time.sleep(.5)
