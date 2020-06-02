#! /usr/bin/env python3


import os
import time
from datetime import datetime
from os.path import expanduser


home = expanduser("~")

flights = set()
locations = []
  
for dirName, subdirList, fileList in os.walk(home):
    subdirList = sorted(subdirList)
    #print('Found directory: %s' % dirName)
    for s_dir in subdirList:
        if (s_dir.startswith('20200') and (s_dir is not None)):
            if (not s_dir in flights):
                locations.append(os.path.join(dirName, s_dir))
                flights.add(s_dir)

flights = sorted(flights)
with open('flights.csv','w') as f:
#    for item in flights:
#        print(item)
        
    for item in locations:
        date_time_s = item.split("/")[-1:]
        outstring = f'{date_time_s[0]},{item}'
        print(outstring)
        #f.write(item.split('/')
