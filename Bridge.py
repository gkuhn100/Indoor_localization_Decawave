import serial
import time
import datetime


DWM=serial.Serial(port="/dev/ttyACM0", baudrate=115200)
print("Connected to " +DWM.name)
DWM.write("\r\r".encode())
time.sleep(1)
DWM.write("lec\r".encode())
time.sleep(1)
count = 0
init = False
pc = time.time_ns()

def print_anchor(Line):
    line = Line.decode('ascii')
    Anch_name =  []
    Anch_place = []
    line = line.split(",")
    num_anchor = int(line[line.index("DIST")+1])       
    for place,item in enumerate(line):
        if item.find("AN") !=-1:
            Anch_name.append(item)
            Anch_place.append(place)

    print('There are {0} Anchors ' .format(num_anchor))
    for i in range(len(Anch_place)):
        print("Anchor {0} is named {1} At located at {2} {3} {4}".format(Anch_name[i],Line[Anch_place[i]+1],Line[Anch_place[i]+2],Line[Anch_place[i]+2],Line[Anch_place[i]+3]))

def time_delta():
    pt = time.time_ns()
    tc = time.time_ns()
    dt = tc - pt
    dt_m = dt * 1e-6
    dt_s = dt * 1e-9
    print("The name elapsed in nano seconds is {0} milli is {1} and in seconds is {2} ".format(dt,dt_m,dt_s ))
    pt = tc
    return(dt_s)  
pt = time.time_ns()
print(pt)
while True:
    try:
        line=DWM.readline()
        if(line):
            if len(line)>=140:
                tc= time.time_ns()
                dt = tc -pt
                pt = tc
                #print_anchor(line)
                count +=1
                print(count)
                parse=line.decode().split(",")
                x_pos=parse[parse.index("POS")+1]
                y_pos=parse[parse.index("POS")+2]
                qf = parse[parse.index("POS")+4]
                val = (x_pos,y_pos)
                print('At time ' + datetime.datetime.now().strftime("%H:%M:%S"),"(",x_pos,",",y_pos,")")
            else:
                print("Position not calculated: ",line.decode())
    except Exception as ex:
        print(ex)
        break

DWM.write("\r".encode())
DWM.close()

