## Imported modules
import serial
import time
import datetime
import multiprocessing as mp
from sense_hat import SenseHat
sense = SenseHat()

## Global Variables
count = 0
init = False

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
    X = round(accel['x'],3)
    Y = round(accel['y'],3)
    Accel = [X,Y]
    return(Accel)

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

## Sorts the Results of the command after "lec" has been entered
def sort_lec(line):
    global count
    line = line.decode()
    line = line.split()
    if len(line) > 10 and line.find('DIST') != -1:
        count +=1
        for place,item in enumerate(line):
            if item.find('POS') != -1:
                pos = place + 1
                tag_lec = line[pos:]
                return(tag_lec)
    else:
        return(False)

## Function which displays if the position of the anchor
def print_anchor(line):
    global count
    Anch_name =  []
    Anch_place = []
    line = line.decode()
    line = lec_pos.split(",")
    num_anchor = line[1]
    print(f"There are {num_anchor} Anchors in the setup")
    for place,item in enumerate(line):
        if item.find != -1:
            Anch_name.append(item)
            Anch_place.append(place)
    for i in range(len(Anch_place)):
        print("Anchor {0} is named {1} and located at {2} {3} {4}".format(Anch_name[i],line[Anch_place[i]+1],line[Anch_place[i]+2],line[Anch_place[i]+3],line[Anch_place[i]+4]))
    print()
    return(num_anchor)

## Function to determine if the tag node is indeed stationary
def det_stationary(tag_lec, tag_apg, Accel):
    if (tag_lec):
        tag_pos = 1
    else:
        tag_pos = 0.0
    return(tag_pos)

if __name__ == "__main__":
    while True:
        tag_lec = tag2.readline()
        lec_pos = sort_lec(tag_lec)
        time_now = datetime.datetime.now().strftime("%H:%M:%S")
        q  = mp.Queue()
        p1 = mp.Process(target = print_apg(q))
        p1.start()
        p1.join()
        if count == 3 and init == True:
            print_anchor(lec_pos)
            init = False
        while q.empty() is False:
            tag_apg = q.get()
            tag_apg = tag_apg.decode('ascii')
        if len(tag_apg) > 20 and tag_apg.find('apg') != -1:
            tag_loc,qf = sort_apg(tag_apg)
            accel = get_accel()
            dT = time.time() - dT
            print(f"At time {time_now} the tag estimate is {tag_loc} and accelerating at {accel} m/s^2")
            time.sleep(1)
