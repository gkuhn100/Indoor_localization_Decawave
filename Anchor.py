#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Aug 28 21:57:16 2021

@author: pi
"""

line = "DIST,4,AN0,0E15,1.75,4.55,2.33,1.61,AN1,1516,0.00,4.55,2.20,2.15,AN2,41B1,1.75,0.00,2.20,4.23,AN3,089F,0.00,0.00,2.20,4.37,POS,1.49,4.02,0.78,59"

if line.find("DIST") != -1:
    Line = line.split(",")
    for place,item in enumerate(Line):
        if item.find("AN") !=-1:
            print(Line[place])
            
        
