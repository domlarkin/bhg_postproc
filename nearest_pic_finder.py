#! /usr/bin/env python3
"""
This module finds the lwir image that is the nearest match for the rgb image.
It creates a new csv file with the data entries for those files.
"""

import os
import sys
import time
import shutil
from os.path import expanduser
from datetime import datetime

def fname_2_time(ftime):
    try:
        utc_time = datetime.strptime(ftime, "%Y%m%d_%H%M%S_%f")
        epoch_time = (utc_time - datetime(1970, 1, 1)).total_seconds()
    except:
        return 404 # Error, Solution Not Found
    return epoch_time

in_filename='/home/user1/Data/20210527_170237_545/20210527_170237_545_boson.csv'
lwir_lines=[]
with open(in_filename, 'r') as f:
    lwir_lines=f.readlines()

in_filename='/home/user1/Data/20210527_170237_545/20210527_170237_545_flir.csv'
bfly_lines=[]
with open(in_filename, 'r') as f:
    bfly_lines=f.readlines()

out_filename='/home/user1/Data/20210527_170237_545/20210527_170237_545_flir_FILTERED.csv'
if os.path.exists(out_filename):
    os.remove(out_filename)

j = 1
prvs_delta = 0
for i in range(1,len(bfly_lines)):
    line=bfly_lines[i]
    bfly_time = fname_2_time(line.split(',')[1])
    while True:
        line=lwir_lines[j]
        lwir_time = fname_2_time(line.split(',')[1])
        if (lwir_time >= bfly_time):
            crnt_delta = abs(lwir_time-bfly_time)
            if(prvs_delta < crnt_delta):
                #print("Using Previous",prvs_time,bfly_time,lwir_time)
                out_line=lwir_lines[j-1]
            else:
                #print("Using Current",prvs_time,bfly_time,lwir_time)
                out_line=lwir_lines[j]
            break # Exit once nearest line is found
        prvs_delta = crnt_delta
        j += 1
        if (j > len(bfly_lines)):
            out_line = "END OF LWIR FILE, NO MATCH FOUND"
            break # Exit if at end of file and no match found

    with open(out_filename, 'a') as f:
        f.write(out_line)   



'''
# SLOW Solution below here
def nearest(items, pivot):
    return min(items, key=lambda x: abs(x - pivot))

# Get only lwir times that match RGB times        
in_filename='/home/user1/Data/20210527_170237_545/20210527_170237_545_boson.csv'
lwir_times=[]
with open(in_filename, 'r') as f:
    for line in f: 
        lwir_time = fname_2_time(line.split(',')[1])
        lwir_times.append(lwir_time)
#print(lwir_times)

# Get RGB Times, we only want lwir times that match these
in_filename='/home/user1/Data/20210527_170237_545/20210527_170237_545_flir.csv'
both_times=[]
with open(in_filename, 'r') as f:
    for line in f: 
        rgb_time = fname_2_time(line.split(',')[1])
        mtch_time = nearest(lwir_times, rgb_time)
        both_times.append((rgb_time,mtch_time,rgb_time-mtch_time))

for item in both_times:
    print(item)
'''


