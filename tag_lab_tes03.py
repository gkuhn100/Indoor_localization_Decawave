
"""
Created on Sun Jun 26 16:59:03 2022

This file is used to test ability of  only one Decawave's tag's position to be 
tracked and predicted with a kalman filter. Connect to the tag pi is a piHat
and one Decawave Tag
 
@author: gkuhn
"""

## Imported modules
import serial
import time
import datetime
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
LOS = False 


## Establish a serial coonection
baudrate = 115200
port1 = "/dev/ttyAMA0"
tag1 = serial.Serial(port1, baudrate, timeout = 1) ##tag_apg
time.sleep(1)

## Tag1 is used for the "apg" command. Solely to get the position of the tag
if tag1.isOpen:
    print(f"{tag1.name} is open and connected to port1 ")
    tag1.write("\r\r".encode())
    
## Function to print the Acceleration of the tag in the x and y coordinates
def get_accel():
    accel = sense.get_accelerometer_raw()
    X = accel['x'] * 9.81 - 0.062
    X = round(X,3)
    Y = accel['y'] * 9.81 + 0.968
    Y = round(Y,3)
    Accel = [X,Y]
    return(Accel)

## enters command 'apg' and gets the results returned from this line
def print_apg():
    tag1.write("apg\r".encode())
    tag_pos = tag1.readline()
    return tag_pos

## Sorts the Results of the command after "apg" has been entered
def sort_apg(tag_pos):
    global count
    apg_tag = tag_pos.decode()
    apg_line = apg_tag.split(',')
    if len(apg_line) > 10 and apg_tag.find('DIST') != -1:
        count +=1

if __name__ == "__main__":
    while True:
        time_now = datetime.datetime.now().strftime("%H:%M:%S")
        print(time_now)
        accel = get_accel()
        print(accel)
        tag_apg = print_apg()
        time.sleep(1)

