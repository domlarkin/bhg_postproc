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
import csv

def set_datadir():
    home = expanduser("~")
    data_dir = home + "/Data/"  
    print("===== The data directory is: %s =====" % data_dir) 
    return data_dir    

# Return the start and end times of an ardupilot log file (BIN file)
def get_bin_times(filename):
    notimestamps = False
    planner_format = True
    m_type=['AHR2']
    mlog = mavutil.mavlink_connection(filename, planner_format,
                                      notimestamps,
                                      robust_parsing=True) 
    start_time = None
    stop_time = None                                   
    while True: 
        m = mlog.recv_match(condition=None, type=m_type, blocking=True)
        if m is None:
            break # Exit the while loop when done with BIN file
        if start_time is None or start_time > m._timestamp:
            start_time = m._timestamp
        if stop_time is None or stop_time < m._timestamp:
            stop_time = m._timestamp
    return (start_time,stop_time)

# Create a string with the start and stop times of each ardupilot log file (BIN file) in this directory 
# Write the string to file and return the string   
def make_bin_times_csv(bin_dir):
    binfile_list = []
    for dirName, subdirList, fileList in os.walk(bin_dir):
        for fname in fileList:
            bin_filename = os.path.join(dirName,fname)
            if bin_filename.endswith('BIN'):
                binfile_list.append(bin_filename)

    binfile_list.sort()  
    csv_filename = bin_dir + "/logfile_times.csv"
    with open(csv_filename, 'w') as csvfile:    
        for bin_filename in binfile_list:
            (begin_time,end_time) = get_bin_times(bin_filename)    
            if begin_time is None or end_time is None:
                print(f"File {bin_filename} has no begin or end time.")
                continue
            human_begin = "%s.%02u" % (time.strftime("%Y-%m-%d,%H:%M:%S",
                                      time.gmtime(begin_time)),
                                      int(begin_time*100.0)%100)
            human_end = "%s.%02u" % (time.strftime("%Y-%m-%d,%H:%M:%S",
                                      time.gmtime(end_time)),
                                      int(end_time*100.0)%100)                              

            #outstring += f"{bin_filename.split('/')[-1]},{begin_time},{end_time},{human_begin},{human_end}\n"
            outstring = f"{bin_filename},{begin_time},{end_time},{human_begin},{human_end}"
            csvfile.write(outstring+'\n')
            print(outstring)
    return (csv_filename)

def convert_bin(filename): 
    '''
    Gets the AHR2 messages from an Ardupilot BIN file and converts into a csv file.     
    filename: The BIN file to read in.    
    The output is written to csv at the same location and filename as the BIN, except 
    the file extension is changed from BIN to csv.
    returns the filename of the saved file
    '''
#    log_dir = '/home/user1/PC21_Collect/Archive/HarveyLogs/APM/LOGS/'
#    filename = log_dir + "00000010.BIN"
    notimestamps = False
    planner_format = True
    m_type=['AHR2']
    mlog = mavutil.mavlink_connection(filename, planner_format,
                                      notimestamps,
                                      robust_parsing=True)    
#    csv_filename = f'{log_dir}00000010.csv'  
    csv_filename = filename.split('.')[0] + ".csv"
    print("csv_filename",csv_filename)
    csvfile=open(csv_filename, 'w')
    with open(csv_filename, 'w') as csvfile:
        csv_columns = ['TimeStamp', 'HumanTime', 'mavpackettype', 'Type', 'Lat', 'Lng', 'Alt', 'Roll', 'Pitch', 'Yaw', 'Q1', 'Q2', 'Q3', 'Q4', 'TimeUS']
        writer = csv.DictWriter(csvfile, fieldnames=csv_columns)
        writer.writeheader()
        while True: 
            m = mlog.recv_match(condition=None, type=m_type, blocking=True)
            if m is None:
                break # Exit the while loop when done with BIN file
            m_dict=m.to_dict()
            #print(m_dict)
            human_time = "%s.%02u" % (time.strftime("%Y-%m-%d %H:%M:%S",
                                      time.gmtime(m._timestamp)),
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

# Return the closest item to a pivot
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
def get_flightbin_json(bin_times_file, csv_file):
    notimestamps = False
    planner_format = True
    m_type=['AHR2']
    mlog = mavutil.mavlink_connection(bin_times_file, planner_format,
                                      notimestamps,
                                      robust_parsing=True) 
    csv_entries=[]
    with open(csv_file, 'r') as f:
        csv_entries=f.readlines()  
    csv_entries = csv_entries[5:]

    test_csvtime = csv_entries.pop(0).split(',')[1]

    logs=[]
    last_ahr2 = None
    while True:
        m = mlog.recv_match(condition=None, blocking=True) # gets the next line in the BIN file
        if m is None: # Exit the loop when at the end of the pixhawk log file
            print('----- At end of BIN file ----- ')
            break #TODO Load next log file  
        m_dict=m.to_dict() 
        print(m._timestamp)
        log_dt = datetime.utcfromtimestamp(m._timestamp) # Convert to a datetime object

        # Add time stamps to the dictionary. All times are in UTC.
        human_time = "%s.%02u" % (time.strftime("%Y-%m-%d %H:%M:%S",
                                  time.gmtime(m._timestamp)),
                                  int(m._timestamp*100.0)%100)
        m_dict['TimeStamp']=m._timestamp
        m_dict['HumanTime']=human_time

        # Replace NaN with None to allow for json parser to make valid json and then create a list of log entries to write to file  
        logs.append(ast.literal_eval(str(m_dict).replace('nan','None')))           

        #print("===",test_csvtime)            
        utc_time = datetime.strptime(test_csvtime, "%Y%m%d_%H%M%S_%f")
        csv_epoch_time = (utc_time - datetime(1970, 1, 1)).total_seconds()
        
        # If the message is AHR2 message then parse it for interpolation
        if (m_dict['mavpackettype'] == 'AHR2'):
            if (last_ahr2 is not None): 
                print("{:.3f}, {:.5f}, {:.5f}".format(last_ahr2['TimeStamp'],csv_epoch_time, m_dict['TimeStamp']))

                interpolated_msg = parse_ahr2(last_ahr2, m_dict,csv_epoch_time)
                if(interpolated_msg is None):  
                    last_ahr2 = m_dict
                    continue
                else:
#                    print(last_ahr2)
#                    print(interpolated_msg)
                    print(test_csvtime)
                    try:
                        test_csvtime = csv_entries.pop(0).split(',')[1]
                    except IndexError as e:
                        print('----- At end of CSV file ----- ')                        
                    #TODO write new values out to csv file
            else:
                last_ahr2 = m_dict

    # Write to a json file all Pixhawk log entries that apply to this flight.  
    json_outiflename = csv_file.split('.')[:-1][0] + '.json'    
    with open(json_outiflename, 'w') as fout:
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
#        print("\n\n Found it =====      ALTS:", last_msg['Alt'],return_msg['Alt'],crnt_msg['Alt'],pct)
        print("\nFound it =====   {:.3f}  ALTS:{:.3f}, {:.3f}, {:.3f}, {:.3f}".format(csv_time,last_msg['Alt'],return_msg['Alt'],crnt_msg['Alt'],pct))
    return return_msg

# Find and delete any empty files. Sometimes the code creates empty image files.
def remove_empty_files(top_dir):
    empty_files = []
    for dirName, subdirList, fileList in os.walk(top_dir):
        for fname in fileList:
            full_filename = os.path.join(dirName,fname)
            if (os.path.getsize(full_filename) < 1):
                empty_files.append(full_filename)
                print("Removing empty file at: %s" %full_filename)  
                os.remove(full_filename)
    return empty_files

# Create a list of empty directories.
def delete_empty_folders(dir_path):
    dirs_to_delete = []
    for root, dirs, files in os.walk(dir_path, topdown=False):
        for dirName in dirs:
            full_filename = os.path.join(root,dirName)

            num_files = sum([len(files) for r, d, files in os.walk(full_filename)])
            #print(full_filename,num_files)
            if (num_files < 3):
                dirs_to_delete.append(full_filename)
                print("++++++ DELETING UNUSED FOLDER %s +++++++" % full_filename)
                shutil.rmtree(full_filename) 
    return dirs_to_delete

def process_flights(bin_times_file, data_dir):
    #1 Walk data directory and process each directory
    for dirName, subdirList, fileList in os.walk(data_dir):
        for fname in fileList:
            full_filename = os.path.join(dirName,fname)
            if full_filename.endswith('csv'):
                bin_file = get_bin_file(full_filename, bin_times_file)
                
def get_bin_file(filename, bin_times_file):
    (start_t,stop_t) = get_flight_times(filename)        
    utc_time = datetime.strptime(start_t, "%Y%m%d_%H%M%S_%f")
    s_time = (utc_time - datetime(1970, 1, 1)).total_seconds()     
    utc_time = datetime.strptime(stop_t, "%Y%m%d_%H%M%S_%f")
    e_time = (utc_time - datetime(1970, 1, 1)).total_seconds()
    
    with open(bin_times_file, 'r') as f:
        times = f.readlines() 
        
    for item in times:
#        print(s_time,e_time,item)
        item_list = item.split(',')
        if s_time > float(item_list[1]) and e_time < float(item_list[2]):
            print(f"=====\nUsing {item_list[0]} to process {filename}.") 
            get_flightbin_json(item_list[0],filename)
            break       
    

if __name__ == '__main__':

    # 1. Walk directories and remove any empty files

    # 2. Walk directories and remove empty directories

    # 3. Create a csv file with a list of ardupilot BIN file start and stop times.
    binlog_dir = '/home/user1/Desktop/Yuma202201HarveyLogs/APM/LOGS/'
#    bin_times_csv_filename = make_bin_times_csv(binlog_dir) # Only need to run this once
    bin_times_csv_filename = binlog_dir + 'logfile_times.csv' # after the above line runs I use this one.

    # 4. Walk directory and find flight csv file and interpolate position data and create json of BIN file data for this flight
#    flight_dir = '/home/user1/Data/20220131_185934_917/'
    flight_dir = '/home/user1/Data/20220125_191846_355/'
    process_flights(bin_times_csv_filename,flight_dir)



'''
        # {'mavpackettype': 'AHR2', 'TimeUS': 618102902, 'Roll': -1.04, 'Pitch': -6.8500000000000005, 'Yaw': 258.05, 'Alt': 165.41000366210938, 'Lat': 32.8470934, 'Lng': -114.2651712, 'Q1': 0.6281170845031738, 'Q2': -0.052161648869514465, 'Q3': -0.030532440170645714, 'Q4': -0.7757678627967834, 'TimeStamp': 1643138757.445591, 'HumanTime': '2022-01-25 19:25:57.44'}

'''






