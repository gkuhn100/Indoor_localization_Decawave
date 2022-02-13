## Imported modules
import serial
import time
import datetime
import multiprocessing as mp
import numpy as np
from sense_hat import SenseHat
sense = SenseHat()

## Global Variables
count = 0 ## counter variable incremented when the "tag_lec" has detected something
dT = 0  ## time elapsed netween tag_apg
delta_X = 0.5 ##Uncertainty in X_position
delta_Y = 0.5 ##Uncertainty in Y_position
init = False ## variable that is initliazed once the print_anchor has been called
iteration = 0 


## Matrices used in the Kalman Filter
A = np.array([[1,0],[0,1]]) # matrix for converting state model matrix
At= np.transpose(A) ## Transpose matrix
B = np.array([[.5,0],[.5,0]],dtype=float) # B matrix for converting control matrix
W = np.array([[.05],[0.025]])# Predict State error matrix
Q = np.array([[.000212],[.04]]) #Error in the Predict State Matrix
R = np.array([[.05],[.05]]) #Measurment Uncertainty Matrix
I = np.array([[1,0],[0,1]]) #Identity Matrix
H = np.array([[1,0],[0,1]]) ##Kalman Gain Conversion Matrix
C = np.array([[1,0],[0,1]]) ##Measurement to Observation matrix
Pc = np.array([[(delta_X * delta_X),0.0], [0.0,delta_Y*delta_Y]], dtype=float) ## initiliaize the Process Covariance Matrix


## Establish a serial coonection
baudrate = 115200
port1 = "/dev/ttyACM0"
port2 = "/dev/ttyACM1"
tag1 = serial.Serial(port1, baudrate, timeout = 1) ##tag_apg
tag2 = serial.Serial(port2, baudrate, timeout = 1) ##lec
time.sleep(1)

## Tag1 is used for the "apg" command. Solely to get the position of the tag
if tag1.isOpen:
    print(f"{tag1.name} is open and connected to port1 ")
    tag1.write("\r\r".encode())

## Tag2 is used for the "lec" command which returns position of both tag and anchor Nodes
if tag2.isOpen:
    print(f"{tag2.name} is open and connected to port2 ")
    tag2.write("\r\r".encode())
    time.sleep(1)
    tag2.write("lec\r".encode())
print('')

## Function to print the Acceleration of the tag in the x and y coordinates
def get_accel():
    accel = sense.get_accelerometer_raw()
    X = accel['x'] * 9.81 - 0.062
    X = round(X,3)
    Y = accel['y'] * 9.81 + 0.968
    Y = round(Y,3)
    Accel = [X,Y]
    return(Accel)


## Sorts the Results of the command after "lec" has been entered
def sort_lec(tag_lec):
    global count
    lec_tag = tag_lec.decode()
    lec_line = lec_tag.split(',')
    if len(lec_line) > 10 and lec_tag.find('DIST') != -1:
        count +=1


## Function which displays if the position of the anchor
def print_anchor(tag_lec):
    Anch_name =  []
    Anch_place = []
    lec_tag = tag_lec.decode()
    lec_pos = lec_tag.split(",")
    num_anchor = int(lec_pos[1])
    print(f"There are {num_anchor} Anchors in the setup")
    for place, item in enumerate(lec_pos):
        if item.find("AN") !=-1:
            Anch_name.append(item)
            Anch_place.append(place)
            print(f"The tag {item} named {lec_pos[place+1]} is at location {lec_pos[place+2]}, {lec_pos[place+3]}, {lec_pos[place+4]}" )

## enters command 'apg' and gets the results returned from this line
def print_apg(q):
    tag1.write("apg\r".encode())
    line = tag1.readline()
    q.put(line)

## Sorts the results from the apg command; provided it was previosuly decoded. returns tag and Quality Factor
def sort_apg(line):
    Line = line.split(" ")
    Line = Line[1:]
    X_pos = float((Line[0].strip('x:'))) * 1e-3
    Y_pos = float((Line[1].strip('y:'))) * 1e-3
    Qf =    (Line[3].strip('qf:'))
    tag_apg = [X_pos,Y_pos]
    return(tag_apg, Qf)


## Function to predict the state of the tag based on its previous state estimate and acceleration
def predict_state(X_est, Accel):
    X_est = np.dot(A,X_est) + np.dot(B,Accel)
    return(X_est)

## Function to predict the state of the tag based on its previous state estimate and accelera
def predict_cov(Pc):
    Pc = np.dot(A,Pc)
    Pc = np.dot(Pc,At) + Q
    return(Pc)


## Function to calculate the Kalman Gain
def Kalman_Gain(X_est,Pc):
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
        ## The sequence ofcode below calls the apg_return function and decodes the tag_position
        q  = mp.Queue() 
        p1 = mp.Process(target = print_apg(q))
        p1.start()
        p1.join()
        while q.empty() is False:
            tag_apg = q.get()
            tag_apg = tag_apg.decode('ascii')      
            
        if init == False: ## This is done simply to get the position of nodes from "lec" command
            tag_lec = tag2.readline()
            sort_lec(tag_lec)        
            
        ## Once there have been three succesfully iterations print anchor and cancel the "lec"
        if count == 3 and init == False:
             print_anchor(tag_lec)
             tag2.write("lec\r".encode())
             print("\n")
             init = True   
        #Now that init is true the stuff begins as normal
        if init == True:
            if len(tag_apg) > 20 and tag_apg.find('apg') !=-1:
                current_time = time.time()
                if iteration == 0:
                   tag_loc,qf = sort_apg(tag_apg)
                   X_est = tag_loc
                   print(f"At time {time_now} the observed tag position is {tag_loc} and is accelerating at {accel} m/s^2\n")
                else:
                     X_est = predict_state(X_est,accel)
                     print(f"At time {time_now} the predicted state is {X_est} and is accelerating at {accel} m/s^2 ")
                     Pc = predict_cov(Pc)
                     Kg = Kalman_Gain(X_est,Pc)
                     tag_loc,qf = sort_apg(tag_apg)
                     X_est = update_state(X_est,tag_loc,Kg)
                     print(f"The observed state is {tag_loc}")
                     print(f"The update state is {tag_loc}")
                     Pc = update_PC(Pc,Kg)
                time.sleep(1)
                dT = round((time.time() - current_time),3)
                iteration += 1
