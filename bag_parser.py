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
                cmd = 'rosbag reindex ' + full_fn + ';rosbag fix --force ' + full_fn + " " + full_fn.split('.')[0] + '.' + full_fn.split('.')[1]
                print(cmd)
                os.system(cmd)

def get_auto_times(inpath,outfile):
    outstring = 'Status,Filename,Crnt Mode, Last Mode, E Time,HR Time\n'
    with open(outfile,'w') as f:
        f.write(outstring)
    for dirName, subdirList, fileList in os.walk(inpath):
        for fname in fileList:
            last_mode = '' 
            if fname.endswith('.bag'):
                bag_file = os.path.join(dirName, fname)
                for topic, msg, t in rosbag.Bag(bag_file).read_messages():
                    if topic == "/mavros/state":                        
                        hr_time = time.strftime("%Y-%m-%d %H:%M:%S.", time.localtime(t.to_time())) + str(t.nsecs)[:2]
                        outstring = f'{bag_file},{msg.mode},{last_mode},{t.to_time()},{hr_time}\n' 
                        if msg.mode == 'AUTO.MISSION' and last_mode != 'AUTO.MISSION':
                            outstring = "STARTED MISSION," + outstring 
                            with open(outfile,'a') as f:
                                f.write(outstring)
                            print(outstring)
                        elif msg.mode != 'AUTO.MISSION' and last_mode == 'AUTO.MISSION':
                            outstring = "STOPED MISSION," + outstring 
                            with open(outfile,'a') as f:
                                f.write(outstring)
                            print(outstring)
                        last_mode = msg.mode

if __name__ == '__main__':
    rootDir = '/home/user1/DATA_ARCHIVE/YUMA'
    csv_file = '/home/user1/DATA_ARCHIVE/YUMA/automode_times.csv'
    rootDir = '/media/user1/SSD4/BHG_Data'
    csv_file = '/media/user1/SSD4/BHG_Data/automode_times.csv'


    # fix_active_bagfiles should be run at least once on a directory
    fix_active_bagfiles(rootDir)
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
