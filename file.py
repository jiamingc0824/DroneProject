#!/usr/bin/env python
import datetime

currentDT = datetime.datetime.now()

f = open("YawandBattery.txt", "a+")
print (str(currentDT))
g = "Stuff" 
f.write(str(currentDT) + "\n")