"""
Created on Sun Jun 26 16:59:03 2022
This file is used to test ability of only one serially connected Decawave tag's
position to be tracked and predicted with a kalman filter.
Connect to the tag pi is a piHat and one Decawave Tag
@author: gkuhn
"""

""" Importing modules and libraries """
import serial
import time
import datetime
import numpy as np
from sense_hat import SenseHat
sense = SenseHat()
np.set_printoptions(formatter={'float': lambda x: "{0:0.5f}".format(x)})


""" Below are the arrays that will be used for Kalman Filtering """
A = np.array([[1,0],[0,1]]) # A matrix for converting state model
At= np.transpose(A) # Transpose of matrix A
B = np.array([[.5,0],[.5,0]],dtype=float) # B matrix for converting control matrix
W = np.array([[.05],[0.025]])# Predict State error matrix
Q = np.array([[.000212],[.04]]) #Error in the Predict State Matrix
R = np.array([[.05],[.05]]) #Measurment Uncertainty Matrix
Pc = np.array([[.05,0.0],[0.0,.05]])#Process Uncertainty Matrix
I = np.array([[1,0],[0,1]]) # Identity Matrix
H = np.array([[1,0],[0,1]]) #Kalman Gain Conversion Matrix
C = np.array([[1,0],[0,1]]) #Measurement to Observation matrix

""" Setting Global Variables """
dT   =  0 # Variable to measure the time elapsed between tag detections
count = 0 # Variable that increases when tag is succesfull detected
iterat  = 0 # How many times the code runs
init  = False # Variable that is used to initialize the code; true after it has been detected at three consecutive time
NLOS   = False # Variable to check if tag is in NLOS or not
delta_X = .05 # Initial Uncertaintity for x position
delta_Y = .05 # Initial Uncertaintity for y position
G = 9.8065 # Converting Gforce to m/s^2
tag_loc_list = []# list to contain all the observed tag_loc; useful in determining if tag is stationary
Qf_list = [] #list which contains all the qualtity factor values
G_Force_X = [] # list which contains all recorded G_forces experienced by tag in X_dimension
G_Force_Y = [] # list which contains all recorded G_forces experienced by tag in Y_dimension
G_2_force_x = 0 # initiliazing X value of G_force
G_2_force_y = 0 # initiliazing X value of G_force
temp = False # used for acceleration

""" Establish a serial connection between tag and Pi """
baudrate = 115200
port1 = "/dev/ttyACM0"
ser = serial.Serial(port1, baudrate, timeout = 1)
time.sleep(1)

if ser.isOpen():
    print("Connected to " + ser.name)
    time.sleep(1)
    line = ser.write("\r\r".encode())
    time.sleep(1)

""" Returns acceleration of tag node in X, Y coordinate in M/S^2 """
def get_accel():
    global iterat
    global G_Force_X
    global G_Force_Y
    global temp
    global G_2_force_x
    global G_2_force_y

    Accel = sense.get_accelerometer_raw()
    X = round(Accel['x'] * G,3)
    Y = round(Accel['y'] * G,3)
    if iterat < 10:
        G_Force_X.append(X)
        G_Force_Y.append(Y)

    elif iterat == 10 and temp == False:
            temp = True

    elif iterat == 10 and temp == True:
            G_2_force_x = sum(G_Force_X) / len(G_Force_X)
            G_2_force_y = sum(G_Force_Y) / len(G_Force_Y)     
        
    elif (iterat >  10) and (temp == True):
        X = X - G_2_force_x
        Y = Y - G_2_force_y

    Accel_list = [X,Y]
    return(Accel_list)

""" Retuns the orientation of X,Y coordinates in radians/s """
def get_gyro():
    gyro = sense.get_gyroscope_raw()
    X = round(gyro['x'], 5)
    Y = round(gyro['y'], 5)
    gyro_list = [X,Y]
    return gyro_list

# f/n print_tag_pos()]
""" This function enters the command apg in the tag node and returns
the decoded value of the line provided the length of the returned values
is greater than 20. If so iteration is "iterated" by one
 """
# return line (decoded into ascii values)
def print_tag_pos():
    global Qf
    ser.write("apg\r".encode())
    line = ser.readline()
    line = line.decode('utf-8')
    if len(line) > 20 and (line.find("x") !=-1) and line.find("y") !=-1:
        return line
    else:
        return None

# f/n sort_qf(line)
"""
only if the length is greater than 20
returns the qualitfy factor, qf
"""
# input line
def sort_qf(line):
    global NLOS
    global Qf
    global iterat
    global Qf_list
    Line = line.split(" ")
    Line = Line[1:]
    Qf = int((Line[3].strip('qf:')))
    Qf_list.append(Qf)
    if iterat > 10:
        if (Qf_list[iterat-1]) and (Qf_list[iterat-2]) and (Qf_list[iterat-3])  == 0:
            NLOS = True
        else:
            NLOS = False

# f/n tag_decode(line)
"""
only if the length is greater than 20
returns the observed,parsed,adjusted position of the tag node
"""
# input line
# return tag_loc
def tag_decode(line):
    global Qf
    global iterat
    Line = line.split()
    Line = Line[1:]
    X_pos = round(float((Line[0].strip('x:'))) * 1e-3 + .05,4)
    Y_pos = round(float((Line[1].strip('y:'))) * 1e-3 + .05,4)
    tag_loc  = [X_pos, Y_pos]
    if Qf == 0 and iterat <=10:
        iterat=0
    else:
        iterat+=1
    return tag_loc

# f/n predict_state(X_est,Accel)
"""
Predicts the state of the tag based on previous positon and acceleration
If it is the first time running the code then inital position estimate is the
observed tag_loc
"""
# input X_est, Accel
# return X_est
def predict_state(X_est, Accel):
    global dT
    B = np.array([[.5*(dT*dT),0],[0,.5*(dT*dT)]],dtype=float) # B matrix for converting control matrix matrix
    X_est = np.dot(A,X_est) + np.dot(B,Accel)
    return(X_est)

"""
For the first iteration of the process covariance matrix
"""
# f/n init_cov()
# return Pc
def init_cov():
    global Pc
    Pc_0 = Pc[0][0] * Pc[0][0]
    Pc_1 = Pc[1][1] * Pc[1][1]
    Pc = np.array([[Pc_0,0],[0,Pc_1]])
    return Pc

# f/n predict_cov(Pc)
""""
Predicts the process covaraince matrix
Remember initial Process covariance is decided beforehand and listed above
"""
# input Pc
# return Pc
def predict_cov(Pc):
    global NLOS
    if NLOS == True: ## if tag passes out of LOS this code will reset the Pc to original value
        Pc = np.array([[(delta_X * delta_X),0.0], [0.0,delta_Y*delta_Y]], dtype=float)## option to set Pc last Pc value
    else:
        Pc = np.dot(A,Pc)
        Pc = np.dot(Pc,At) + Q
        Pc[0][1] = 0.0
        Pc[1][0] = 0.0
    return(Pc)

# f/n kalman_gain(X_est,Pc)
""""
Adjust the Kalman Gain
"""
# input X_est,Pc
# return Kg
def kalman_gain(X_est,Pc):
    global NLOS
    if NLOS == True: # if tag passes out of LOS Reset Kalman Gain to original value
        Kg = np.array([[.833,0.0],[0.0,.833]]) ##  or can set Kg to last kalma gain value
        Kg_temp = np.array([[0.0,0.0],[0.0,0.0]])
        return Kg_temp
    else:
        Kg_num = np.dot(Pc,H)
        Kg_den = np.dot(H,Pc)
        Kg_den = np.dot(Kg_den,H) + R
        Kg     = np.divide(Kg_num,Kg_den)
        Kg[0][1] = 0.0
        Kg[1][0] = 0.0
        return(Kg)

# f/n update_state(X_est,tag_apg,Kg)
""""
Update the estimated state of the tag based on predicted state, observed state and
Kalman Gain. Note remeber the difference between predicted and observed state
"""
# input X_est,tag_apg,Kg
# return X_est
def update_state(X_est,tag_apg,Kg):
    num = tag_apg - np.dot(H,X_est)
    X_est = X_est + np.dot(Kg,num)
    return(X_est)

# f/n update_Pc(Pc,Kg)
""""
Updates the Process covariance matrix prior to process beginning anew
"""
# input Pc, Kg
# return Pc
def update_PC(Pc,Kg):
    num = I - (np.dot(Kg,H))
    Pc = np.dot(num,Pc)
    Pc[0][1] = 0.0
    Pc[1][0] = 0.0
    return(Pc)

# f/n det_stat(tag_loc,Accel)
""" deterimines if the tag is stationary or not """
def det_stat(tag_loc,Accel):
    global tac_loc_list
    global iterat
    global Qf
    global stat
    tag_loc_list.append(tag_loc)
    length = len(tag_loc_list)
    if iterat > 10 and length>1:
        diff_pos_X = tag_loc_list[iterat-1][0] - tag_loc_list[iterat-2][0] # difference between the last two locations of tag in the X_coordinate
        diff_pos_Y = tag_loc_list[iterat-1][1] - tag_loc_list[iterat-2][1] # difference between the last two locations of tag in the X_coordinate
        if (abs(diff_pos_X) < .03 and abs(diff_pos_Y) < .03 and abs(Accel[0]) < .05  and abs(Accel[1]) <.05):
            print("Device is stationary")
            stat = True
        else:
            stat = False

if __name__ == "__main__":
    tag_loc_list = []
    i = 0
    while True:
        time_now = datetime.datetime.now().strftime("%H:%M:%S")
        accel = get_accel()
        tag_pos = print_tag_pos() # The tag position identically observed by Decawave
        try:
            if tag_pos is not None: # Check to ensure command 'apg' returns a valid ouput
                sort_qf(tag_pos) # gets quality factor
                tag_loc = tag_decode(tag_pos) # Decodes and ouputs X,Y coordinate provide tag_pos is valied
                tag_loc_list.append(tag_loc)
                print(f"At time {time_now}, iteration {iterat} the tag is at observed position {tag_loc} and accelerating at {accel}m/s^2 with a quality factor of {Qf}")
                if iterat == 10 and (NLOS == False):# Used to set the initial tag_location NLOS isn't neccesarry
                        Pc = init_cov()
                        delta_t = [time.time()]
                        X_est = tag_loc
                elif iterat > 10:
                        det_stat(tag_loc,accel)
                        delta_t.append(time.time())
                        dT = round(delta_t[iterat-10] - delta_t[iterat - 11],4)
                        if stat == True:
                            print(f"As the AV is stationary the tag's position remains estimated at {X_est}")
                        else:
                            X_est = predict_state(X_est,accel)
                            if NLOS == False:
                                Pc = predict_cov(Pc)
                                print(f"The predicted position is {X_est} with a process covariance of {Pc}")
                                Kg = kalman_gain(X_est,Pc)
                                X_est = update_state(X_est,tag_loc_list[iterat-1],Kg)
                                Pc = update_PC(Pc,Kg)
                                print(f"The kalman gain is {Kg} and the updated position is {X_est} with a updated pc of {Pc} and dt of {dT}")
                            else:
                                print(f"Warning the tag has passed out of the LOS! The Kalman Gain remains {Kg} The Pc is still {Pc} and the Predicted state is {X_est} with a dt of {dT}")
                                ## consider resetting the Kalman gain and process covaraince values differently
        except KeyboardInterrupt:
                print('Error! Keyboard interrupt detected, now closing ports! ')
                ser.close()
        time.sleep(.5)
