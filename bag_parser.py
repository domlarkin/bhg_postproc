# -*- coding: future_fstrings -*-
from __future__ import absolute_import
from __future__ import unicode_literals
import future_fstrings

"""
This module parses ros bag files and finds the start and stop times of missions.
This is written to a csv file along with the time in human readable form.
 
Uses fstrings from Python 3. For this to work in Python2 and ROS you need the 
future commands above and to install below
sudo -H pip2 install future-fstrings

"""
import os

import time
from os.path import expanduser
from datetime import datetime
import rosbag

def fix_active_bagfiles(path):
    for dirName, subdirList, fileList in os.walk(path):
        for fname in fileList:
            #print(os.path.join(dirName, fname))
            if fname.endswith('.active'):
                full_fn = os.path.join(dirName, fname)
                cmd = 'rosbag reindex ' + full_fn + ';rosbag fix ' + full_fn + " " + full_fn.split('.')[0] + '.' + full_fn.split('.')[1]
                print(cmd)
                os.system(cmd)

def get_auto_times(inpath,outfile):
    outstring = 'Status,Filename,E Time,HR Time\n'
    with open(outfile,'w') as f:
        f.write(outstring)
    print(outstring)
    for dirName, subdirList, fileList in os.walk(inpath):
        for fname in fileList:
            last_mode = ''
            last_alt = 0.0 
            if fname.endswith('.bag'):
                bag_file = os.path.join(dirName, fname)
                print(f'Parsing the file: {bag_file}')
                for topic, msg, t in rosbag.Bag(bag_file).read_messages():                        
                    hr_time = time.strftime("%Y-%m-%d %H:%M:%S.", time.localtime(t.to_time())) + str(t.nsecs)[:2]
                    outstring = f'{bag_file},{t.to_time()},{hr_time}' 
                    if topic == "/mavros/state":
                        if msg.mode == 'AUTO.MISSION' and last_mode != 'AUTO.MISSION':
                            outstring = f"STARTED MISSION,{outstring},{msg.mode},{last_mode}"
                            with open(outfile,'a') as f:
                                f.write(outstring+'\n')
                            print(outstring)                            
                        elif msg.mode != 'AUTO.MISSION' and last_mode == 'AUTO.MISSION':                        
                            outstring = f"STOPED MISSION,{outstring},{msg.mode},{last_mode}"  
                            with open(outfile,'a') as f:
                                f.write(outstring+'\n')
                            print(outstring)
                        if(last_mode != msg.mode):
                            print(msg.mode,last_mode)
                        last_mode = msg.mode

                    if topic == "/mavros/altitude":  
                        if msg.relative > 5.0 and last_alt <= 5.0:
                            outstring = f"Climbed above 5M,{outstring},{msg.relative},{last_alt}"  
                            with open(outfile,'a') as f:
                                f.write(outstring+'\n')
                            print(outstring)                            
                        if msg.relative < 5.0 and last_alt >= 5.0:
                            outstring = f"Dropped below 5M,{outstring},{msg.relative},{last_alt}"  
                            with open(outfile,'a') as f:
                                f.write(outstring+'\n')
                            print(outstring)
#                        if(msg.relative > 1.0):
#                            print(msg.relative,last_alt)
                        last_alt = msg.relative
                                                                      
if __name__ == '__main__':
    rootDir = '/home/user1/DATA_ARCHIVE/BHGTest'
    csv_file = '/home/user1/DATA_ARCHIVE/BHGTest/automode_times.csv'
    # fix_active_bagfiles should be run at least once on a directory
    #fix_active_bagfiles(rootDir)
    get_auto_times(rootDir,csv_file)

'''
Start recording when mode begins with AUTO, stop when not AUTO
rostopic echo /mavros/state

header: 
  seq: 210
  stamp: 
    secs: 1583182056
    nsecs: 797319429
  frame_id: ''
connected: True
armed: True
guided: True
manual_input: False
mode: "AUTO.MISSION"
system_status: 4
=========================
TODO Use this to find altitude
rostopic echo /mavros/altitude 

header: 
  seq: 2103
  stamp: 
    secs: 1583182057
    nsecs: 269441664
  frame_id: "map"
monotonic: 204.560455322
amsl: 169.468978882
local: 19.09633255
relative: 18.7274188995
terrain: nan
bottom_clearance: nan



'''        
