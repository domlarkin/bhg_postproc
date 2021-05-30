#! /usr/bin/env python3
"""
This module manages the directories for BHG. It does the following:
    - Creates the data directory and mission directory for saving the images
    - Publishes the mission directory for the cameras to use for saving images
    - Publishes the number of files in the subdirectories
    - 
    
Dependencies:
 sudo -H pip3 install pymavlink
 sudo -H pip3 install pandas 
"""

import os
import time
import shutil
from os.path import expanduser
from pymavlink import mavutil
from datetime import datetime

def set_datadir():
    home = expanduser("~")
    data_dir = home + "/Data/"  
    print("===== The data directory is: %s =====" % data_dir) 
    return data_dir    

def convert_bin(filename): 
    '''
    Gets the AHR2 messages from an Ardupilot BIN file and converts into a csv file.     
    filename: The BIN file to read in.    
    The output is written to csv at the same location and filename as the BIN, except 
    the file extension is changed from BIN to csv.
    returns the filename of the saved file
    '''
    log_dir = '/home/user1/PC21_Collect/Archive/HarveyLogs/APM/LOGS/'
    filename = log_dir + "00000010.BIN"
    notimestamps = False
    planner_format = True
    m_type=['AHR2']
    mlog = mavutil.mavlink_connection(filename, planner_format,
                                      notimestamps,
                                      robust_parsing=True)    
    csv_filename = f'{log_dir}00000010.csv'
    csvfile=open(csv_filename, 'w')
    with open(csv_filename, 'w') as csvfile:
        csv_columns = ['TimeStamp', 'HumanTime', 'Type', 'Lat', 'Lng', 'Alt', 'Roll', 'Pitch', 'Yaw', 'Q1', 'Q2', 'Q3', 'Q4', 'TimeUS']
        writer = csv.DictWriter(csvfile, fieldnames=csv_columns)
        writer.writeheader()
        while True:
            m = mlog.recv_match(condition=None, type=m_type, blocking=True)
            if m is None:
                break # Exit the while loop when done
            m_dict=m.to_dict()            
            human_time = "%s.%02u" % (time.strftime("%Y-%m-%d %H:%M:%S",
                                      time.localtime(m._timestamp)),
                                      int(m._timestamp*100.0)%100)
            m_dict['TimeStamp']=m._timestamp
            m_dict['HumanTime']=human_time
            writer.writerow(m_dict)
    return csv_filename

def make_datetime(name):
    '''
    Converts the date time in the filename into 2 outputs. 
    Output 1 is the epoch time for mathmatical comparison
    Output 2 is a human readable string that matches the one used by ardupilot
    '''
    fname_split=name.split('_')    
    m_sec = round(int(fname_split[3][:3])/10.0)
    if m_sec < 10:
        m_sec = f'{m_sec}0'
    outstring=f'{fname_split[1][:4]}-{fname_split[1][4:6]}-{fname_split[1][6:8]} {fname_split[2][:2]}:{fname_split[2][2:4]}:{fname_split[2][4:6]}.{m_sec}'

    #print(outstring) # 2020-05-28 18-02-08.44
    utc_time = datetime.strptime(outstring, "%Y-%m-%d %H:%M:%S.%f")
    epoch_time = (utc_time - datetime(1970, 1, 1)).total_seconds()
    return (epoch_time,outstring)

def get_datetime_filename(name):
    # FLIR18285440_20210525_151126_537938.png
    start_pos = name.find('_')
    sp=name.find('_')+1
    sp_time=name[sp:].split('.')[0] 
    datetime_object = datetime.strptime(sp_time, '%Y%m%d_%H%M%S_%f')
    return datetime_object

def nearest(items, pivot):
    return min(items, key=lambda x: abs(x - pivot))

def get_filenames(loc):
    filenames=[]
    for dirName, subdirList, fileList in os.walk(loc):
        for fname in fileList:
            full_filename = os.path.join(dirName,fname)
            print("Found empty file at: %s", full_filename)
            filenames.append(full_filename)
    return filenames

def get_file_lists(filenames):
    ir_files=[]
    vs_files=[]
    for item in filenames:
        if('BOSON' in item):
            ir_files.append(item)
        elif('FLIR' in item):
            vs_files.append(item)
    return(vs_files,ir_files)

if __name__ == '__main__':
    rootDir = set_datadir() + '20200528_180208_387820/'
    rootDir = '/home/user1/DATA_ARCHIVE/BHGTest'
    rootDir = set_datadir()


