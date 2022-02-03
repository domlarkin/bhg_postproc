#! /usr/bin/env python3
"""
This module looks thru all ardupilot bin log files for data entries during a specified flight.
It then creates a csv file with all log entries for the flight.
    - 
    
Dependencies:
 sudo -H pip3 install pymavlink
 sudo -H pip3 install pandas 
"""

# 1. Open the csv file and get start time and end time of flight
# 2. Go to the BIN directory and parse each log file looking for data entries during the flight.
# 3. If data entry is found then write it to a csv file.

import os
import time
from os.path import expanduser
from datetime import datetime
import csv
from pymavlink import mavutil

# Create a list of log files with start and end time, sorted by start time.
# Params:
#       log_dir - The directory to recursively search for files ending in BIN.
# Returns:
#       A sorted list of lists. Each list include start time, end time, log file name.
#       Sorted by start time.
def make_bin_list(directory): 
    logfile_list=[]
    for root, directories, files in os.walk(directory):
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
    return logfile_list

def make_datetime_from_filename(name):
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

def get_file_lists(filenames):
    ir_files=[]
    vs_files=[]
    for item in filenames:
        if('BOSON' in item):
            ir_files.append(item)
        elif('FLIR' in item):
            vs_files.append(item)
    return(vs_files,ir_files)

def create_csv_timed(start_time, end_time):
    pass
    
             
def convert_bin_2_csv(filename, makeall = True): 
    '''
    Converts an Ardupilot BIN file into a csv file. 
    makeall if true converts all message types.
    makeall if false converts only message types listed in the m_type list
    The output is in the form of:
    Timestamp epoch seconds, timestamp human readable, the message.
    '''
    # TODO accept log dir as parameter
    # TODO walk the directory for all BIN files

    notimestamps = False
    planner_format = True
    #m_type=['GPA', 'GPS', 'AHR2', 'POS']
    m_type=['GPS']
    mlog = mavutil.mavlink_connection(filename, planner_format,
                                      notimestamps,
                                      robust_parsing=True)    
    #csv_filename = f'{log_dir}00000010.csv'
    csv_filename = filename.split(".")[0]+'.csv'
    
    if(not makeall):
        csv_filename = filename.split(".")[0]+'_loc.csv'

    message_types= set()
    with open(csv_filename, 'w') as f:
        while True:
            outstring = ''
            m = mlog.recv_match(condition=None, blocking=True)
            if(not makeall):
                m = mlog.recv_match(condition=None, type=m_type, blocking=True)

            if m is None:
                break
            else:
                message_types.add(m.get_type())
            if True:
                if notimestamps:
                    outstring = f'{m}'
                else:
                    # TODO format the CSV better
                    #print(m.to_dict().keys())
                    outstring = "%s,%s.%02u," % (m._timestamp,
                        time.strftime("%Y-%m-%d %H:%M:%S",
                        time.localtime(m._timestamp)),
                        int(m._timestamp*100.0)%100)
                    print(type(m.to_dict()),m.to_dict())
                    exit()
                    outstring += f'{m["Lng"]}'
                f.write(f'{outstring}\n')
    
                
def convert_gps_bin(filename): 
    '''
    Gets the AHR2 messages from an Ardupilot BIN file and converts into a csv file.     
    filename: The BIN file to read in.    
    The output is written to csv at the same location and filename as the BIN, except 
    the file extension is changed from BIN to csv.
    returns the filename of the saved file
    '''
    notimestamps = False
    planner_format = True
    m_type=['GPS']
    mlog = mavutil.mavlink_connection(filename, planner_format,
                                      notimestamps,
                                      robust_parsing=True)    
    csv_filename = filename.split(".")[0]+'_GPS.csv'
    csvfile=open(csv_filename, 'w')
    with open(csv_filename, 'w') as csvfile:
        csv_columns = ['TimeStamp', 'HumanTime', 'mavpackettype', 'Lat', 'Lng', 'Alt', 'Yaw', 'Spd', 'TimeUS', 'Status', 'GMS', 'GWk', 'NSats', 'HDop', 'GCrs', 'VZ', 'U']
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
                
                
                
                
                
if __name__ == "__main__":
#    log_dir = "/home/user1/PC21_Collect/Archive/HarveyLogs/APM/LOGS"
#    
#    with open("BIN_file_times.csv", "w", newline="") as f:
#        writer = csv.writer(f)
#        writer.writerow(["Start Time","End Time","Filename"])
#        writer.writerows(make_bin_list(log_dir))
#        
#    print("Results written to BIN_file_times.csv")    
#    
#    rootDir = '/media/user1/SSD4/BHG_Data'
#    csv_file = '/media/user1/SSD4/BHG_Data/automode_times.csv'
#    # fix_active_bagfiles should be run at least once on a directory
#    fix_active_bagfiles(rootDir)
#    get_auto_times(rootDir,csv_file)


    log_dir = '/home/user1/PC21_Collect/Archive/HarveyLogs/APM/LOGS/'
    filename = log_dir + "00000010.BIN"
    convert_gps_bin(filename)



'''
AHR2 {TimeUS : 2955690875, Roll : -18.26, Pitch : -42.21, Yaw : 62.31, Alt : 4.670000076293945, Lat : 41.3906091, Lng : -73.95326349999999, Q1 : 0.817762017250061, Q2 : 0.05731012672185898, Q3 : -0.38087040185928345, Q4 : 0.427689790725708}

POS {TimeUS : 2955590728, Lat : 41.390607599999996, Lng : -73.9532641, Alt : 4.739999771118164, RelHomeAlt : 0.21198183298110962, RelOriginAlt : 0.3019818365573883}

GPA {TimeUS : 2955677829, VDop : 1.25, HAcc : 2.07, VAcc : 3.58, SAcc : 0.41000000000000003, YAcc : 0.0, VV : 1, SMS : 2955648, Delta : 200}

GPS {TimeUS : 2955677829, Status : 3, GMS : 339130000, GWk : 2107, NSats : 14, HDop : 0.77, Lat : 41.3906091, Lng : -73.95326349999999, Alt : 2.2600000000000002, Spd : 0.9170000553131104, GCrs : 224.25595092773438, VZ : -0.12300000339746475, Yaw : 0.0, U : 1}
'''        
