import serial
import time
import datetime


DWM=serial.Serial(port="/dev/ttyACM0", baudrate=115200)
print("Connected to " +DWM.name)
DWM.write("\r\r".encode())
time.sleep(1)
DWM.write("lec\r".encode())
time.sleep(1)
time_delta = 0
count = 0
Init = False
while True:
    try:
        line=DWM.readline()
        if(line):
            if len(line)>=140:
                count +=1
                parse=line.decode().split(",")
                x_pos=parse[parse.index("POS")+1]
                y_pos=parse[parse.index("POS")+2]
                qf = parse[parse.index("POS")+4]
                val = (x_pos,y_pos)
                print('At time ' + datetime.datetime.now().strftime("%H:%M:%S"),"(",x_pos,", ",y_pos,")")
                if count > 5:
                    Init = True
            else:
                print("Position not calculated: ",line.decode())
                count = 0
    except Exception as ex:
        print(ex)
        break

DWM.write("\r".encode())
DWM.close()



