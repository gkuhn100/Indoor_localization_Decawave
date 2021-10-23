import serial
import time
import datetime
#from sense_hat import SenseHat
#sense = SenseHat()

DWM = serial.Serial(port="COM5", baudrate=115200, timeout=1)
print('Connected to ' + DWM.name)
DWM.write("\r\r".encode())
time.sleep(1)
DWM.write("lec\r".encode())
time.sleep(1)
init = False
count = 0
stat =  0


def get_accel():
    accel = sense.get_accelerometer()
    X = accel['X']
    Y = accel['Y']
    Accel_list = [X,Y]
    return (Accel_list)

def det_motion(Accel_list):
    X_accel = Accel_list[0]
    Y_accel = Accel_list[1]

def print_kalman(Acce_list,tag_loc):
    print("position")

def print_anchor(Line):
    Anch_name =  []
    Anch_place = []
    line = Line.decode('ascii')
    print(line)
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
    else:
        print('hi')

while True:
    #Accel=get_accel()
    line=DWM.readline()
    if(line):
        print('Count is {0}'.format(count))
        if len(line)>=140:
            count +=1
            if count == 1:
                print_anchor(line)
            if count == 5:
                init = True
            if init:
                stat = 0
                parse=line.decode().split(",")
                x_pos=parse[parse.index("POS")+1]
                y_pos=parse[parse.index("POS")+2]
                qf = parse[parse.index("POS")+4]
                val = (x_pos,y_pos)
                print('At time ' + datetime.datetime.now().strftime("%H:%M:%S") + ' The Tag is at location' ,"(",x_pos,",",y_pos,")"+ ' with a quality factor of', qf)
        else:
            print("Position not calculated: ",line.decode())
    if not line and init == True:
        print('device is not moving')
        stat = 1
    if stat == 1:
        DWM.write("lec\r".encode())
        Line = DWM.readline()
        Line = Line.decode('ascii')
        time.sleep(.1)
        if Line.find("APG") != -1:
            X_pos = float(line[0].strip('x:'))*1e-3
            Y_pos = float(parse[1].strip('y:'))*1e-3
            print('device is stationary')


DWM.write("\r".encode())
DWM.close()
