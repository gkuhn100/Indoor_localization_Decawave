# -*- coding: utf-8 -*-
"""
Created on Sat Apr 30 20:53:47 2022

@author: GregK
"""
import serial
import time
from datetime import datetime
import pandas as pd

baudrate = 115200
port1 = "COM4"
Time = 0

lines =    []
x_pos =    []
y_pos =    []
z_pos =    [] 
qf    =    [] 
tag_name = []
elapsed =  []
date =     []

tag1 = serial.Serial(port1, baudrate, timeout = 1)


if tag1.isOpen:
    tag1.write("\r\r".encode())
    time.sleep(1)


while(1):
    tag1.write("lec\r".encode())
    line = tag1.readline().decode('utf-8')
    print(line)
    ##print(f'The line {line} is of length {len(line)}   ')
    line_encode = line.split(",")
    lines.append(line_encode)
    if len(line) >= 30:    
        for place,item in enumerate(line_encode):
            if item.find('POS')!=-1:
                x_pos.append(line_encode[place+3])
                y_pos.append(line_encode[place+4])
                z_pos.append(line_encode[place+5])
                qf.append(line_encode[place+6])
                tag_name.append(line_encode[place+7])
                date.append(datetime.now())
    Time+=1
    elapsed.append(Time)
    time.sleep(.5)
    

df = pd.DataFrame('X_Pos': [x_pos],'Y_Pos':[y_pos], 'qf':[qf],
                  'tag_name':[tag_name],'Datetime':[date])
