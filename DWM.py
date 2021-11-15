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

### Global Variables
count  = 0
init   = False
anchor = False

##Establishes a serial connection with the decawave tag_module
ser =  serial.Serial('/dev/ttyACM0',115200,timeout=1)

if ser.isOpen:
    print(f'Connected to port {ser.name} ' )
    time.sleep(1)
    ser.write('\r\r'.encode())
    time.sleep(1)
    ser.write('lec\r'.encode())

#Prints the name and Position of the Anchor nodes. Will be called just once
def print_anchor(Line):
    Anch_name =  []
    Anch_place = []
    line = Line
    Line = line.split(",")
    num_anchor = Line[1]
    print("There are {0} anchors in this Setup".format(num_anchor))
    for place,item in enumerate(Line):
        if item.find("AN") !=-1:
            Anch_name.append(item)
            Anch_place.append(place)
    for i in range(len(Anch_place)):
        print("Anchor {0} is named {1} and located at {1} {2} {3}".format(Anch_name[i],Line[Anch_place[i]+1],Line[Anch_place[i]+2],Line[Anch_place[i]+2],Line))
    print()
    return(num_anchor)

## Function to get the accleration of the tag node
def get_accel():
    accel = sense.get_accelerometer_raw()
    X = accel['x']
    Y = accel['y']
    X = round(X,3)
    Y = round(Y,3)
    Accel_list = [X,Y]
    return (Accel_list)

##Function to determine the quality factor of the Tag and therefore determine if
## The Tag is out of the LOS
def get_qf(line):
    line = line.split(",")
    num_pos = len(line)
    qf = line[num_pos-1]
    return(qf)

def get_anchor(line,num_anchor):
    Line = line.split(",")
    anchor_total = Line[1]
    diff_anchor = anchor_total - num_anchor
    if (anchor_total - num_anchor) > 0:
        print('warning')
    else:
        print('idk')
    return(diff_anchor)
## Gets the position and quality factor of the tag node using 'apg' command
def tag_apg():
    ser.write('apg/r'.encode())
    line = ser.readline()
    line = line.decode('ascii')
    return(line)

##Function to return the position of the tag from the lec output
def tag_lec(line):
    pos = 0
    line = line.split(",")
    #print(line)
    for place,item in enumerate(line):
        if item.find("POS") != -1:
            pos = place + 1
            tag_pos = line[pos:]
    return(tag_pos)
    

##Function to predict the state of tag 
def predict_state(tag,Accel_list):
    state = tag[0:1]
    state_est = np.dot(A,state) + np.dot(B,Accel_list)
    return(state_est)

   ## 
def Kalman():
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

if __name__ == '__main__':
    while(1):
        time_now = time.strftime("%H:%M:%S")
        line = ser.readline()
        line = line.decode('ascii')
        if len(line) >=140 and line.find("DIST")!=-1:
            count+=1
            if count == 1:
                anchor_init = print_anchor(line)
            elif count>=3:
                 Line = tag_lec(line)
                 anchor_num = get_anchor(line,anchor_init)
                 x_pos_lec = Line[0]
                 y_pos_lec = Line[1]
                 qf = get_qf(line)
                 #print(f'The tag is at position {x_pos_lec},{y_pos_lec}, with a quality factor of {qf} at time {time_now}' )
                 init = True
        
