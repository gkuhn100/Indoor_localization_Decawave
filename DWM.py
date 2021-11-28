## Imported modules
import serial
import time
import datetime
import numpy as np
from sense_hat import SenseHat
sense = SenseHat()

## Matrices used in the Kalman Filter
A = np.array([[1,0],[0,1]]) # A matrix for converting state model
At= np.transpose(A)
B = np.array([[.5,0],[.5,0]],dtype=float) # B matrix for converting control matrix
W = np.array([[.05],[0.025]])# Predict State error matrix
Q = np.array([[.000212],[.04]]) #Error in the Predict State Matrix
R = np.array([[.05],[.05]]) #Measurment Uncertainty Matrix
I = np.array([[1,0],[0,1]]) #Identity Matrix
H = np.array([[1,0],[0,1]]) ##Kalman Gain Conversion Matrix
C = np.array([[1,0],[0,1]]) ##Measurement to Observation matrix
delta_X = .5 ##Uncertainty in X_position
delta_Y = .5 ##Uncertainty in Y_position

## Code to open and establish the serial port connection
DWM = serial.Serial("/dev/ttyACM0", 115200, timeout = 1)
time.sleep(1)
print('Connected to ' + DWM.name)
print()
DWM.write("\r\r".encode())
time.sleep(1)
DWM.write("lec\r".encode())
time.sleep(1)
init = False
Temp = False
count = 0

## The Function below will be used to recieve and return the tag's acceleration
def get_accel():
    accel = sense.get_accelerometer_raw()
    X = accel['x']
    Y = accel['y']
    X = round(X,3)
    Y = round(Y,3)
    Accel_list = [X,Y]
    return (Accel_list)

## The Function below is used to get the Quantity,Name,and Location of the Anchor Nodes
def print_anchor(Line):
    Anch_name =  []
    Anch_place = []
    if Line.find("DIST") != -1:
        line = Line.split(",")
        num_anchor = line[1]
        print("There are {0} anchors in this Setup".format(num_anchor))
        for place,item in enumerate(Line):
            if item.find("AN") !=-1:
                Anch_name.append(item)
                Anch_place.append(place)
        for i in range(len(Anch_place)):
            print("Anchor {0} is named {1} At located at {1} {2} {3}".format(Anch_name[i],Line[Anch_place[i]+1],Line[Anch_place[i]+2],Line[Anch_place[i]+2],Line[Anch_place[i]+3]))
        print()
        return(num_anchor)

## This function is used to Parse through line after command "apg" has been entered"
def get_tag(Line):
    Line = Line.split()
    Line = Line[1:]
    X_pos = (Line[0].strip('x:'))
    Y_pos = (Line[1].strip('y:'))
    Qf =    (Line[3].strip('qf:'))
    tag = [X_pos+.05,Y_pos+.05]
    return(tag,Qf)

## Function to return and print the tag's position from the 'lec' command
def tag_lec(Line):
    length = len(Line)
    line = Line.split(',')
    for place,item in enumerate(line):
        if item.find("POS") != -1 and (length)>140:
            pos = place + 1
            tag_pos = line[pos:]
            print(tag_pos)
            return(tag_pos)

## Function to predict the future State
def predict_state(x_est,Accel_list):
    Accel = np.array([[Accel_list[0]],[Accel_list[1]]],dtype=float)
    X_est = np.dot(A,est) + np.dot(B,Accel)+np.ndarray(W)
    return(X_est)

def proccess_cov_int():
    Pc = np.array([[delta_X*delta_X], delta_X*delta_Y], [[delta_Y*delta_X],delta_Y*delta_Y],dtype=float)
    Pc[0][1] = 0.0
    Pc[1][0] = 0.0
    return(Pc)

def proccess_cov(Pc):
    Pc = np.dot(A,Pc)
    Pc = np.dot(Pc,At)+Q
    Pc[0][1] = 0.0
    Pc[1][0] = 0.0
    return(Pc)


## Displays an error signal if one of the anchor node is malfunctioning
def anchor_error(line,num_anchor):
    Line = line.split(",")
    if len(line) > 140 and line.find('DIST') != -1:
        anchor_total = Line[1]
        anchor_total = int(anchor_total)
        diff_anchor = anchor_total - num_anchor
        if (anchor_total - num_anchor) > 0:
            print('warning')
        return(diff_anchor)

## Main Function
if __name__ == '__main__':
    dT = 0.0
    PC = proccess_cov_int()
    while True:
        time_now = datetime.datetime.now().strftime("%H:%M:%S")
        line = DWM.readline()
        line = line.decode('ascii')
        Accel = get_accel()
        if line.find('DIST') != -1:
            count+=1
            tag_loc=tag_lec(line)
            print(f'At time {time_now} the tags estimated position is {tag_loc} and is accelerating at {Accel}m/s^2')
        if count == 1 and not Temp:
            anchor_init=int(print_anchor(line))
            Temp = True
        if count == 3:
            init = True
        if init:
            DWM.write('apg\r'.encode())
            time.sleep(.5)
            dT = time.time() - dT
            num_anchor=anchor_error(line,anchor_init)
            if line.find("apg") != -1 and len(line)>10:
                tag_loc,Qf = get_tag(line)
                est = predict_state(est,Accel)
                Pc = proccess_cov(Pc)
                print("At time {0} the Tag's observed location is {1} with a Quality factor of {2} and Accelerating at {3} m/s^2" .format(time_now,tag_loc,Qf,Accel))
                print('The Predicited State is ')
                print('The Kalman Gain is ')
                print('The Updated State is ')

DWM.write("\r".encode())
DWM.close()
