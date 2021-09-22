import serial
import time
#import datetime


DWM=serial.Serial(port="/dev/ttyACM0", baudrate=115200)
print("Connected to " +DWM.name)
DWM.write("\r\r".encode())
time.sleep(1)
DWM.write("lec\r".encode())
time.sleep(1)
init = False
count = 0

def print_anchor(Line):
    Anch_name =  []
    Anch_place = []
    line = Line.decode('ascii')
    print(line)
    if line.find("DIST") != -1:
        Line = line.split(",")
        num_anchor = int(Line[Line.index("DIST")+1])
        print("There are {0} anchors in this Setup".format(num_anchor))
        for place,item in enumerate(Line):
            if item.find("AN") !=-1:
                Anch_name.append(item)
                Anch_place.append(place) 
        for i in range(len(Anch_place)):
            print("Anchor {0} is named {1} At located at {1} {2} {3}".format(Anch_name[i],Line[Anch_place[i]+1],Line[Anch_place[i]+2],Line[Anch_place[i]+2],Line[Anch_place[i]+3]))

while True:
    try:
        line=DWM.readline()
        print_anchor(line)
        if(line):
            if len(line)>=140:
                count +=1
                if count > 5:
                    parse=line.decode().split(",")
                    x_pos=parse[parse.index("POS")+1]
                    y_pos=parse[parse.index("POS")+2]
                    qf = parse[parse.index("POS")+4]
                    val = (x_pos,y_pos)
                    #print('At time ' + datetime.datetime.now().strftime("%H:%M:%S") + ' The Tag is at location' ,"(",x_pos,",",y_pos,")"+ ' with a quality factor of', qf)
                    #anchor_numb = print_anchor(line)
                
            else:
                print("Position not calculated: ",line.decode())
    except Exception as ex:
        print(ex)
        break

DWM.write("\r".encode())
DWM.close()
