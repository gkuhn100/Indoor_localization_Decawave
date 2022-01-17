## Imported modules
import serial
import time
import datetime
import multiprocessing as mp
from sense_hat import SenseHat
sense = SenseHat()

## Establish a serial coonection
baudrate = 115200
port1 = "/dev/ttyACM1"
port2 = "/dev/ttyACM2"
tag1 = serial.Serial(port1, baudrate, timeout = 1) ##tag_apg
tag2 = serial.Serial(port2, baudrate, timeout = 1) ##tag_lec
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
    X_pos = (Line[0].strip('x:'))
    Y_pos = (Line[1].strip('y:'))
    Qf =    (Line[3].strip('qf:'))
    ##Qf = 75
    tag_apg = [X_pos,Y_pos]
    return(tag_apg, Qf)

## Sorts the Results of the command after "lec" has been entered
def sort_lec(line):
    global Count = 0
    line = line.decode()
    line = line.split(',')
    if len(line) > 10 and line.find('DIST') != -1:
        Count +=1
        for place,item in enumerate(line):
            if item.find('POS') != -1:
                pos = place + 1
                tag_lec = line[pos:]
                print(tag_lec)
                return(tag_lec)
    else:
        return(False)

## Function to determine if the tag node is indeed stationary
def det_stationary(tag_lec, tag_apg, Accel):
    if (tag_lec == False):
        tag_pos = 1
    else:
        tag_pos = 0.0
    return(tag_pos)

## Function that displays the position and status of the anchor
def print_anchor(lec_pos):
    Anch_name =  []
    Anch_place = []
    line = lec_pos.split(",")
    num_anchor = line[1]
    print(f"There are {num_anchor} Anchors in the setup")
    for place,item in enumerate(line):
        if item.find('POS') != -1:
            Anch_name.append(item)
            Anch_place.append(place)
    for i in range(len(Anch_place)):
        print("Anchor {0} is named {1} and located at {2} {3} {4}".format(Anch_name[i],line[Anch_place[i]+1],line[Anch_place[i]+2],line[Anch_place[i]+3],line[Anch_place[i]+4]))
    print()
    return(num_anchor)

if __name__ == "__main__":
    dT = 0.0
    Global Count
    count = 0
    while True:
        tag_lec = tag2.readline()
        tag_pos_lec = sort_lec(tag_lec)
        if (tag_pos_lec) != false:
            count +=1
        if count == 1:
            print_anchor()
            print(f"The count is {Count})
        time_now = datetime.datetime.now().strftime("%H:%M:%S")
        q  = mp.Queue()
        p1 = mp.Process(target = print_apg(q))
        p1.start()
        p1.join()
        while q.empty() is False:
            tag_apg = q.get()
            tag_apg = tag_apg.decode('ascii')
        if len(tag_apg) > 20 and tag_apg.find('apg') != -1:
            tag_loc,qf = sort_apg(tag_apg)
            accel = get_accel()
            tag_pos = det_stationary(tag_lec, tag_apg, Accel)
            dT = time.time() - dT
            print(f"At time {time_now} the tag estimate is {tag_loc} and accelerating at {accel} m/s^2")
            time.sleep(1)
