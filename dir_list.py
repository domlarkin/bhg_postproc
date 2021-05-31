#! /usr/bin/env python3

import os
import time


path='/media/user1/BHG_USMA01/'
flights=set()
for dirName, subdirList, fileList in os.walk(path):
    for item in subdirList:
        print(item)
        if('202105' in item):
            flights.add(item)

csv_filename='mission_list.csv'
try:
    os.remove(csv_filename)
except OSError:
    pass
s_flights=sorted(list(flights))
for item in s_flights:
    with open(csv_filename, 'a') as csvfile:
        csvfile.write(item+'\n')


