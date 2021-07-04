#! /usr/bin/env python3
"""
This module looks thru all ardupilot bin log files for data entries during a specified flight.
It then creates a csv file with all log entries for the flight.
    - 
    
Dependencies:
 sudo -H pip3 install pymavlink
 sudo -H pip3 install pandas 
"""

import os
import csv
from pymavlink import mavutil

log_dir = "/home/user1/PC21_Collect/Archive/HarveyLogs/APM/LOGS"

logfile_list=[]
for root, directories, files in os.walk(log_dir):
    for file in files:
        if file.endswith(".BIN"):
            filename = os.path.join(root, file)
            notimestamps = False
            planner_format = True
            mlog = mavutil.mavlink_connection(filename, planner_format,
                                              notimestamps,
                                              robust_parsing=True)       
            m = mlog.recv_match(condition=None, blocking=True)
            start_time = m._timestamp
            while(True):                                              
                m = mlog.recv_match(condition=None, blocking=True)
                if m is None:
                    break
                stop_time = m._timestamp
            print(file)
            outList = [start_time,stop_time,filename]
            logfile_list.append(outList)
logfile_list = sorted(logfile_list, key = lambda x: x[1]) # sort list of BIN files by log start time

#for item in logfile_list:
#    print(item)

with open("BIN_file_times.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["Start","End","Filename"])
    writer.writerows(logfile_list)
    
print("Results written to BIN_file_times.csv")    
    
'''
AHR2 {TimeUS : 2955690875, Roll : -18.26, Pitch : -42.21, Yaw : 62.31, Alt : 4.670000076293945, Lat : 41.3906091, Lng : -73.95326349999999, Q1 : 0.817762017250061, Q2 : 0.05731012672185898, Q3 : -0.38087040185928345, Q4 : 0.427689790725708}

POS {TimeUS : 2955590728, Lat : 41.390607599999996, Lng : -73.9532641, Alt : 4.739999771118164, RelHomeAlt : 0.21198183298110962, RelOriginAlt : 0.3019818365573883}

GPA {TimeUS : 2955677829, VDop : 1.25, HAcc : 2.07, VAcc : 3.58, SAcc : 0.41000000000000003, YAcc : 0.0, VV : 1, SMS : 2955648, Delta : 200}

GPS {TimeUS : 2955677829, Status : 3, GMS : 339130000, GWk : 2107, NSats : 14, HDop : 0.77, Lat : 41.3906091, Lng : -73.95326349999999, Alt : 2.2600000000000002, Spd : 0.9170000553131104, GCrs : 224.25595092773438, VZ : -0.12300000339746475, Yaw : 0.0, U : 1}
'''        
