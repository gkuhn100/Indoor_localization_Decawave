import serial
import time
import datetime


DWM=serial.Serial(port="/dev/ttyACM0", baudrate=115200)
print("Connected to " +DWM.name)
DWM.write("\r\r".encode())
time.sleep(1)
DWM.write("lec\r".encode())
time.sleep(1)
init = False
count = 0
pc = time.time_ns()

def print_anchor(Line):
    line = Line.decode('ascii')
    Anch_name = []
    Anch_place = []

    if line.find("DIST") != -1:
        Line = line.split(",")
        num_anchor = int(Line[Line.index("DIST")+1])
    for place,item in enumerate(Line):
        if item.find("AN") !=-1:
            Anch_name.append(item)
            Anch_place.append(place)

    print("There are {0} anchors in this Setup".format(num_anchor))
    for i in range(len(Anch_place)):
        print("Anchor {0} is named {1} At located at {2} {3} {4}".format(Anch_name[i],Line[Anch_place[i]+1],Line[Anch_place[i]+2],Line[Anch_place[i]+2],Line[Anch_place[i]+3]))

    

while True:
    try:
        line=DWM.readline()
        print_anchor(line)
        if(line):
            if len(line)>=140:
                parse=line.decode().split(",")
                x_pos=parse[parse.index("POS")+1]
                y_pos=parse[parse.index("POS")+2]
                qf = parse[parse.index("POS")+4]
                val = (x_pos,y_pos)
                print_anchor(line)
                print('At time ' + datetime.datetime.now().strftime("%H:%M:%S"),"(",x_pos,",",y_pos,")")
                count +=1
                tc = time.time_ns()
                dt = (tc - pc)* 1e-9
                pc = tc
                
                if count == 5:
                   init = True
              
                
            else:
                print("Position not calculated: ",line.decode())
    except Exception as ex:
        print(ex)
        break

DWM.write("\r".encode())
DWM.close()


