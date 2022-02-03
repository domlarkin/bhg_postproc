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
import json
import ast
import copy

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
    
def get_flight_times(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()        
    start_time = lines[2].split(',')[1]
    stop_time = lines[-1:][0].split(',')[1]
    return (start_time,stop_time)
    
def make_st_datetime(str_time):
    #  convert 20200528_180208_387820 to time data structure.
    datetime_object = datetime.strptime(str_time, '%Y%m%d_%H%M%S_%f')
    return datetime_object
    
# For Ardupilot message details see: https://ardupilot.org/plane/docs/logmessages.html    
def get_flightbin_json(filename):
#    csv_file = '/home/user1/Data/20220125_191846_355/20220125_191846.csv'    
    csv_file = '/home/user1/Data/20220125_181032_725/20220125_181032.csv'
    line = get_flight_times(csv_file) # ('20220125_191846_971', '20220125_192737_914')
    s_time = make_st_datetime(line[0])
    e_time = make_st_datetime(line[1])
    print(s_time,e_time)

    notimestamps = False
    planner_format = True
    m_type=['AHR2']
    mlog = mavutil.mavlink_connection(filename, planner_format,
                                      notimestamps,
                                      robust_parsing=True) 
    logs=[]
    last_ahr2 = None
    while True:
        m = mlog.recv_match(condition=None, blocking=True) # gets the next line in the BIN file
        if m is None: # Exit the loop when at the end of the pixhawk log file
            break 
        m_dict=m.to_dict() 
        log_dt = datetime.utcfromtimestamp(m._timestamp) # Convert to a datetime object

        if (log_dt < s_time) or (log_dt > e_time): # Skip processing if log entry is before the flight begins
            continue

        # Add time stamps to the dictionary. All times are in UTC.
        human_time = "%s.%02u" % (time.strftime("%Y-%m-%d %H:%M:%S",
                                  time.gmtime(m._timestamp)),
                                  int(m._timestamp*100.0)%100)
        m_dict['TimeStamp']=m._timestamp
        m_dict['HumanTime']=human_time
        
        # Replace NaN with None to allow for json parser to make valid json and create a list of log entries to write to file  
        logs.append(ast.literal_eval(str(m_dict).replace('nan','None')))           

        # Get the timestamp for the pictures from the csv file and convert to epoch time
        test_csvtime = '20220125_181144_988' #TODO get this from the csv file one line at a time.
        utc_time = datetime.strptime(test_csvtime, "%Y%m%d_%H%M%S_%f")
        csv_epoch_time = (utc_time - datetime(1970, 1, 1)).total_seconds()
        
        # If the message is AHR2 message then parse it for interpolation
        if (m_dict['mavpackettype'] == 'AHR2'):
            if (last_ahr2 is not None): 
                print(last_ahr2['TimeStamp'],csv_epoch_time, m_dict['TimeStamp'])
                interpolated_msg = parse_ahr2(last_ahr2, m_dict,csv_epoch_time)
                if(interpolated_msg is None):  
                    last_ahr2 = m_dict
                    continue
                else:
                    print(last_ahr2)
                    print(interpolated_msg)
                    print(m_dict)
                    #TODO write new values out to csv file
                    #TODO get the next line from the csv and do it again instead of break
                    break
            else:
                last_ahr2 = m_dict

    # Write to a json file all Pixhawk log entries that apply to this flight.        
    with open(test_csvtime+'.json', 'w') as fout:
        json.dump(logs, fout, indent=2) # indent causes new lines to be created, which makes it easier for text editors to parse
    return logs
    
def bhg_interpolate(last_value, current_value, pcnt):
    adjustment = (current_value - last_value) * pcnt
    return last_value + adjustment   
    
def parse_ahr2(last_msg, crnt_msg,csv_time):
    return_msg = None
    if (last_msg['TimeStamp'] <= csv_time <= crnt_msg['TimeStamp']):
        return_msg = copy.deepcopy(crnt_msg)
        delta_time = crnt_msg['TimeStamp'] - last_msg['TimeStamp'] # Total time between two sensor readings
        diff_csv_time = csv_time - last_msg['TimeStamp'] # Time from last sensor reading to desired measurements
        pct = diff_csv_time / delta_time # Ratio of change in sensor readings that we want to apply
        return_msg['Lat'] = round(bhg_interpolate(last_msg['Lat'],crnt_msg['Lat'],pct),7)
        return_msg['Lng'] = round(bhg_interpolate(last_msg['Lng'],crnt_msg['Lng'],pct),7)
        return_msg['Pitch'] = round(bhg_interpolate(last_msg['Pitch'],crnt_msg['Pitch'],pct),2)
        return_msg['Yaw'] = round(bhg_interpolate(last_msg['Yaw'],crnt_msg['Yaw'],pct),2)
        return_msg['Roll'] = round(bhg_interpolate(last_msg['Roll'],crnt_msg['Roll'],pct),2)
        return_msg['Alt'] = round(bhg_interpolate(last_msg['Alt'],crnt_msg['Alt'],pct),3)
        print("\n\n Found it =====      ALTS:", last_msg['Alt'],return_msg['Alt'],crnt_msg['Alt'],pct)
    return return_msg

if __name__ == '__main__':
#    rootDir = set_datadir() + '20200528_180208_387820/'
#    rootDir = '/home/user1/DATA_ARCHIVE/BHGTest'
#    rootDir = set_datadir()
    binlog_dir = '/home/user1/Desktop/Yuma202201HarveyLogs/APM/LOGS'
#    csv_file = '/home/user1/Data/20220131_185934_917/20220131_185934.csv'
#    csv_file = '/home/user1/Data/20220125_191846_355/20220125_191846.csv'
#    line = get_flight_times(csv_file) # ('20220125_191846_971', '20220125_192737_914')
#    print(line)
#    print(make_st_datetime(line[0]),make_st_datetime(line[1]))

    # 1. Get next flight log csv file
    # 2. Get start and end time of the file
    # 3. Search bin directories for corresponding bin file
    # 4. Save only bin entries during the flight into JSON file 
    # 5. Interpolate position data between last and next point based on time


    bin_filename = '/home/user1/Desktop/Yuma202201HarveyLogs/APM/LOGS/00000077.BIN'
    logs = get_flightbin_json(bin_filename)
    
#    
#    for item in logs:
#        print(item)



'''

        # {'mavpackettype': 'AHR2', 'TimeUS': 618102902, 'Roll': -1.04, 'Pitch': -6.8500000000000005, 'Yaw': 258.05, 'Alt': 165.41000366210938, 'Lat': 32.8470934, 'Lng': -114.2651712, 'Q1': 0.6281170845031738, 'Q2': -0.052161648869514465, 'Q3': -0.030532440170645714, 'Q4': -0.7757678627967834, 'TimeStamp': 1643138757.445591, 'HumanTime': '2022-01-25 19:25:57.44'}

'''






