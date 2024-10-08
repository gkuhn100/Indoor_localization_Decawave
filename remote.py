## Imported modules
import serial
import time
import datetime
import multiprocessing as mp
import numpy as np
from sense_hat import SenseHat
sense = SenseHat()
np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

## Global Variables
count = 0 ## counter variable incremented when the "tag_lec" has detected something
dT = 0  ## time elapsed between tag_apg
delta_X = 0.5 ##Uncertainty in X_position
delta_Y = 0.5 ##Uncertainty in Y_position
init = False ## variable that is initliazed once the print_anchor has been called
iteration = 0 ## Variable used to count the total number of iterations run
LOS = False ## Variable to determine if the tag is out of the LOS we oringally assume it is
Stat = False ## Variable to determine if the tag is stationary or not

## Matrices used in the Kalman Filter
A = np.array([[1,0],[0,1]]) # matrix for converting state model matrix
At= np.transpose(A) ## Transpose
W = np.array([[.05],[0.025]])# Predict State error matrix
Q = np.array([[0.05],[0.05]]) #Error in the Predict State Matrix
R = np.array([[.05],[.05]]) #Measurment Uncertainty Matrix
I = np.array([[1,0],[0,1]]) #Identity Matrix
H = np.array([[1,0],[0,1]]) ##Kalman Gain Conversion Matrix
C = np.array([[1,0],[0,1]]) ##Measurement to Observation matrix
Pc = np.array([[(delta_X * delta_X),0.0], [0.0,delta_Y*delta_Y]], dtype=float) ## initiliaize the Process Covariance Matrix

## Establish a serial connection
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
<<<<<<< Updated upstream
    X = accel['x'] * 9.81 - 0.062
    X = round(X,3)
    Y = accel['y'] * 9.81 + 0.968
    Y = round(Y,3)
=======
    X = round(accel['x'],3) * 9.81
    Y = round(accel['y'],3) * 9.81
>>>>>>> Stashed changes
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
    return(num_anchor)

## enters command 'apg' and gets the results returned from this line
def print_apg(q):
    tag1.write("apg\r".encode())
    line = tag1.readline()
    q.put(line)

## Sorts the results from the apg command; provided it was previosuly decoded. returns tag location
def sort_apg(line):
    Line = line.split(" ")
    Line = Line[1:]
    X_pos = round(float((Line[0].strip('x:'))) * 1e-3 + .05,4)
    Y_pos = round(float((Line[1].strip('y:'))) * 1e-3 + .05,4)
    ##Qf =    (Line[3].strip('qf:'))
    tag_apg = [X_pos,Y_pos]
    return(tag_apg)

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

##Function which runs if and only if an anchor node malfunctions
def missing_anchor(tag_pos,kg,Pc,accel):
    print('Anchor node has malfunctined')
    ## Determine missing Anchor
    ## Determine tags position from missing anchor
    ## If far away do nothing; if closeby do adjust kalman filter and prediction

##Function to determine if the tag is stationary or not; this function will run soley based on
## on the 'apg' tag' node at this time. In the future may include the 'lec' tag

def stationary(tag_apg,accel):
    global iteration
    tag_pos_apg = []
    accel_list = []
    tag_pos_apg.append(tag_apg)
    accel_list.append(accel)

if __name__ == "__main__":
    while True:
        time_now = datetime.datetime.now().strftime("%H:%M:%S")
        accel = get_accel()
        q  = mp.Queue()
        p1 = mp.Process(target = print_apg(q))
        p1.start()
        p1.join()
        while q.empty() is False:
            tag_apg = q.get()
            tag_apg = tag_apg.decode('ascii')
        if init == False:
            tag_lec = tag2.readline()
            sort_lec(tag_lec)
        if count == 3 and init == False:
<<<<<<< Updated upstream
             num_anchor=print_anchor(tag_lec)
=======
             print_anchor(tag_lec)
>>>>>>> Stashed changes
             tag2.write("lec\r".encode())
             print("\n")
             init = True
        if init == True:
            try:
                if len(tag_apg) > 20 and tag_apg.find('apg') !=-1:
                    current_time = time.time()
                    if iteration == 0:
                        qf = sort_qf(tag_apg)
                        tag_loc = sort_apg(tag_apg)
                        X_est = tag_loc
                        print(f"At iteration {iteration} and time {time_now} the observed tag position is {tag_loc} ")
                    else:
                         qf = int(sort_qf(tag_apg))
                         if (qf > 0 ):
                             X_est = predict_state(X_est,accel)
                             predict = X_est
                             tag_loc= sort_apg(tag_apg)
                             Kg = Kalman_Gain(X_est,Pc)
                             X_est = update_state(X_est,tag_loc,Kg)
                             Pc = update_PC(Pc,Kg)
                             print(f"The quality factor is {qf}")
                             print(f"At iteration {iteration} and time {time_now} the predicted state is {predict} and is accelerating at {accel} m/s^2 ")
                             print(f"The observed state is {tag_loc}")
                             print(f"The Kalman Gain is {Kg}")
                             print(f"The updated state is therefore {X_est} ")
                             print(f"The process covariance matrix is {Pc}")
                         elif qf == 0:
                            LOS = True
                            X_est = predict_state(X_est,accel)
                            print(f"Warining the tag node is currently out of the LOS! The estimated position is {X_est} and is accelerating at {accel}m/s^2")
                         #elif num_anchor > current_anchor:
                             #missing_anchor(tag_loc,Kg,accel)

                    time.sleep(1)
                    dT = round((time.time() - current_time),3)
                    iteration += 1
            except KeyboardInterrupt:
                print("Error! keybord interrupt detected, now closing the ports")
                tag1.close()
                tag2.close()
