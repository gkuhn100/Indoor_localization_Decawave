import datetime
import time
import serial
import random

DWM=serial.Serial(port="COM5", baudrate=115200)
print("Connected to " +DWM.name)
DWM.write("\r\r".encode())
time.sleep(1)
init = False
count = 0


def get_accel():
    ax = random.randrange(0, 100, 1) * .002
    ay = random.randrange(0, 100, 1) * .002
    acc = [ax, ay]
    return(acc)
print('There are four anchors in this Setup ')
print('Anchor AN0 is named 41B1 At located at 41B1 0.00 0.00 ')
print('Anchor AN1 is named 0E15 At located at 0E15 1.75 0.00 ')
print('Anchor AN2 is named 089F At located at 089F 1.75 4.32 ')
print('Anchor AN3 is named 1516 At located at 1516 0.00 4.32 ')


while True:
    time_now = time.strftime("%H:%M:%S")
    DWM.write('apg\r'.encode())
    line = DWM.readline()
    Acc = get_accel()

    if 'dwm'.encode() not in line and len(line)>10:
        parse = line.decode().split()
        tag_loc = parse[1:]

        print('At time {0} The node is at position {1} and accelerating at {2}m/s^2 '.format(time_now, tag_loc, Acc))

    time.sleep(.5)




DWM.write("\r".encode())
DWM.close()
