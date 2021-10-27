#libraries and modules to import
import serial
import time
import datetime
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


DWM = serial.Serial('/dev/ttyACM0',115200,timeout = 1)
print('Connected to ' + DWM.name)
print()
DWM.write("\r\r".encode())
time.sleep(1)
DWM.write("lec\r".encode())
time.sleep(1)
anchor = False
init = False
temp = False
count = 0

## The Function below will be used to get the acceleration of the tag
def get_accel():
    accel = sense.get_accelerometer_raw()
    X = accel['x']
    Y = accel['y']
    X = round(X,3)
    Y = round(Y,3)
    Accel_list = [X,Y]
    return (Accel_list)

##Function below is used to get the Quantity,Name, and Location of the Decawave Nodes
def print_anchor(Line):
    Anch_name =  []
    Anch_place = []
    line = Line
    if line.find("DIST") != -1:
        Line = line.split(",")
        num_anchor = Line[1]
        print("There are {0} anchors in this Setup".format(num_anchor))
        for place,item in enumerate(Line):
            if item.find("AN") !=-1:
                Anch_name.append(item)
                Anch_place.append(place)
        for i in range(len(Anch_place)):
            print("Anchor {0} is named {1} At located at {1} {2} {3}".format(Anch_name[i],Line[Anch_place[i]+1],Line[Anch_place[i]+2],Line[Anch_place[i]+2],Line[Anch_place[i]+3]))
    

## Function Below is used to Parse through tag_position
def get_tag(Line):
    Line = Line.split()
    Line = Line[1:]
    X_pos = (Line[0].strip('x:'))
    Y_pos = (Line[1].strip('y:'))
    Qf    = (Line[3].strip('qf:'))
    tag = [X_pos,Y_pos]
    return(tag)

##Function to return the location estimation
def loc_est(tag,Accel_list):
    loc_est = np.dot(A,tag)+np.dot(B,Accel)
    
    return(loc_est)
   
def update_pc(pc):
    pc = np.dot(A,pc)
    pc = np.dot(pc,At)+Q

def Kalman_Gain(pc):
    Kg_num = np.dot(pc,H)
    Kg_den = np.dot(H,pc)
    Kg_den = np.dot(Kg_den,H)+R
    Kg = np.divide(Kg_num,Kg_den)
    Kg[0][1] = 0.0
    Kg[1][0] = 0.0
    print("The Kalman Gain is {0}" .format(Kg))
    return(Kg)
    
def update_state(X_est,Tag_loc,Kg):
    num = Tag_loc - np.dot(H,X_est)
    X_est = X_est + np.dot(Kg,num)
    print()
    print('Updated State')
    print(X_est)
    print()
    return(X_est)
    

while True:
    line = DWM.readline()
    line = line.decode('ascii')
    Accel = get_accel()
    if len(line) > 140 and line.find('DIST') != -1:
        count+=1
    if count == 1 and not anchor:
        print_anchor(line)
        anchor = True
        print("\n")
    elif count == 3:
        init = True
    if init:
        DWM.write('apg\r'.encode())
        time.sleep(.5)
        if line.find("apg") != -1 and len(line)>10:
            tag_loc = get_tag(line)
            print("At time {0} the Tag is at location {1} and Accelerating at {2} m/s^2" .format(datetime.datetime.now().strftime("%H:%M:%S"),tag_loc, Accel))
            est_loc=loc_est(Accel,tag_loc)
            print('The estimated value is {0} ' .format(est_loc))
            
            




DWM.write("\r".encode())
DWM.close()
