## Imported modules
import serial
import time
import datetime
import numpy as np
from sense_hat import SenseHat
sense = SenseHat()

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

## The Function below will be used to get the acceleration of the tag
def get_accel():
    accel = sense.get_accelerometer_raw()
    X = accel['x']
    Y = accel['y']
    X = round(X,3)
    Y = round(Y,3)
    Accel_list = [X,Y]
    return (Accel_list)

##Function below is used to get the Quantity,Name,and Location of the Decawave Nodes
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
        print()
        return(num_anchor)

## Function Below is used to Parse through line
def get_tag(Line):
    Line = Line.split()
    Line = Line[1:]
    X_pos = (Line[0].strip('x:'))
    Y_pos = (Line[1].strip('y:'))
    Qf =    (Line[3].strip('qf:'))
    tag = [X_pos,Y_pos]
    return(tag,Qf)

## function to print the tag for the lec command
def tag_lec(Line):
    line = Line.split(',')
    for place,item in enumerate(line):
        if item.find("POS") != -1 and len(line)>27:
            pos = place + 1
            tag_pos = line[pos:]
            return(tag_pos)

##Displays an error signal if one of the anchor node is malfunctioning
def get_anchor(line,num_anchor):
    Line = line.split(",")
    if len(line) > 120 and line.find('DIST') != -1:
        anchor_total = Line[1]
        anchor_total = int(anchor_total)
        diff_anchor = anchor_total - num_anchor
        return(diff_anchor)
        if (anchor_total - num_anchor) > 0:
            print('warning')

if __name__ == '__main__':
    while True:
        line = DWM.readline()
        line = line.decode('ascii')
        print(line)
        Accel = get_accel()
        if line.find('DIST') != -1:
            count+=1
            lec_pos=tag_lec(line)
            print(f'At time {datetime.datetime.now().strftime("%H:%M:%S")} the tag is at position {lec_pos}')
        if count == 1 and not Temp:
            anchor_init=int(print_anchor(line))
            Temp = True
        if count == 3:
            init = True
        if init:
            DWM.write('apg\r'.encode())
            time.sleep(.5)
            ##get_anchor(line,anchor_init)
            if line.find("apg") != -1 and len(line)>10:
                tag_loc,Qf = get_tag(line)
                print("At time {0} the Tag is at location {1} with a Quality factor of {2} and Accelerating at {3} m/s^2" .format(datetime.datetime.now().strftime("%H:%M:%S"),tag_loc,Qf,Accel))

DWM.write("\r".encode())
DWM.close()
