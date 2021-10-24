import serial
import time
import datetime
import numpy as np
#from sense_hat import SenseHat
#sense = SenseHat()

DWM = serial.Serial(port="COM5", baudrate=115200, timeout=1)
print('Connected to ' + DWM.name)
print()
DWM.write("\r\r".encode())
time.sleep(1)
DWM.write("lec\r".encode())
time.sleep(1)
init = False
temp = False
count = 0

## The Function below will be used to get the acceleration of the tag
def get_accel():
    accel = sense.get_accelerometer()
    X = accel['X']
    Y = accel['Y']
    Accel_list = [X,Y]
    return (Accel_list)

##Function below is used to get the Quantity,Name, and Location of the Decawave Nodes
def print_anchor(Line):
    Anch_name =  []
    Anch_place = []
    line = Line
    if line.find("DIST") != -1:
        Line = line.split(",")
        num_anchor = 4
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
    tag = [X_pos,Y_pos, Qf]
    return(tag)

## Function Below is used to calculate the Kalman Filter
def calc_kalman(Accel_list,tag):
    Loc_est = np.array[1,0]


while True:
    line = DWM.readline()
    line = line.decode('ascii')
    Accel = [X,Y]
    if len(line) > 140 and line.find('DIST') != -1:
        count+=1
        print(line)
    if count == 1 and not temp:
        print_anchor(line)
        Temp = True
    elif count == 3:
        init = True
    if init:
        DWM.write('apg\r'.encode())
        time.sleep(.5)
        if line.find("apg") != -1 and len(line)>10:
            tag_loc = get_tag(line)
            print("At time {0} the Tag is at location {1} and Accelerating at {2} m/s^2" .format(datetime.datetime.now().strftime("%H:%M:%S"),tag_loc,Accel))





DWM.write("\r".encode())
DWM.close()
