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

def convert_bin_2_csv(makeall = True): 
    '''
    Converts an Ardupilot BIN file into a csv file. 
    makeall if true converts all message types.
    makeall if false converts only message types listed in the m_type list
    The output is in the form of:
    Timestamp epoch seconds, timestamp human readable, the message.
    '''
    # TODO accept log dir as parameter
    # TODO walk the directory for all BIN files
    log_dir = '/home/user1/Projects/bhg_postproc/'
    filename = log_dir + "disarm_test.bin"

    notimestamps = False
    planner_format = True
    m_type=['GPA', 'GPS', 'AHR2', 'POS']
    mlog = mavutil.mavlink_connection(filename, planner_format,
                                      notimestamps,
                                      robust_parsing=True)    
    csv_filename = f'{log_dir}00000010.csv'
    if(not makeall):
        csv_filename = f'{log_dir}00000010_loc.csv'

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
                    outstring = "%s,%s.%02u,%s" % (m._timestamp,
                        time.strftime("%Y-%m-%d %H:%M:%S",
                                      time.localtime(m._timestamp)),
                        int(m._timestamp*100.0)%100, m)
                f.write(f'{outstring}\n')

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

def remove_empty_files(path):
    '''
    Search for and delete empty files. The Gobi camera creates empty files if it 
    does not shutdown cleanly.
    '''
    for dirName, subdirList, fileList in os.walk(path):
        for fname in fileList:
            full_filename = os.path.join(dirName,fname)
            if (os.path.getsize(full_filename) < 1):
                print("Removing empty file at: %s" %full_filename)
                os.remove(full_filename)

def find_dirs_to_delete(path):
    '''
    Data directories are created before a mission is started. If the mission is aborted
    or fails to run successfully this function deletes the entire mission directory.
    '''
    #TODO Only create directories when a mission starts.
    #NOTE In later code, the directory cleanup code does this already.
    dirs_to_delete = []
    for root, dirs, files in os.walk(path):
        for folder in dirs:
            check_dir = os.path.join(root, folder)
            num_files = sum([len(files) for r, d, files in os.walk(check_dir)])
            if (num_files < 5): 
                dirs_to_delete.append(check_dir)
            #print(check_dir,num_files)
        break
    return dirs_to_delete
    
def delete_folders(dirs_to_delete):     
    """ Deletes all unused directories except the symlink 'latest' """   
    for folder in dirs_to_delete:
        print("++++++ DELETING UNUSED FOLDER %s +++++++" % folder)
        shutil.rmtree(folder)    

if __name__ == '__main__':
    rootDir = set_datadir() + '20200528_180208_387820/'
    rootDir = '/home/user1/Projects/bhg_postproc'
    #convert_bin_2_csv()
    convert_bin_2_csv(False)
    #remove_empty_files(rootDir)
    #delete_folders(find_dirs_to_delete(rootDir))

#    for dirName, subdirList, fileList in os.walk(rootDir):
#        for fname in fileList:
#            full_filename = os.path.join(dirName,fname)
#            if (os.path.getsize(full_filename) < 10):
#                print("Found empty file at: %s", full_filename)
            #os.remove("ChangedFile.csv")

            #print(f'{fname}')          
#            if ('GOBI' in fname):
#                file_etime, file_stime = make_datetime(fname)
#                print(f'\t{fname}  date {file_stime}  etime: {file_etime}')

'''
AHR2 {TimeUS : 2955690875, Roll : -18.26, Pitch : -42.21, Yaw : 62.31, Alt : 4.670000076293945, Lat : 41.3906091, Lng : -73.95326349999999, Q1 : 0.817762017250061, Q2 : 0.05731012672185898, Q3 : -0.38087040185928345, Q4 : 0.427689790725708}

POS {TimeUS : 2955590728, Lat : 41.390607599999996, Lng : -73.9532641, Alt : 4.739999771118164, RelHomeAlt : 0.21198183298110962, RelOriginAlt : 0.3019818365573883}

GPA {TimeUS : 2955677829, VDop : 1.25, HAcc : 2.07, VAcc : 3.58, SAcc : 0.41000000000000003, YAcc : 0.0, VV : 1, SMS : 2955648, Delta : 200}

GPS {TimeUS : 2955677829, Status : 3, GMS : 339130000, GWk : 2107, NSats : 14, HDop : 0.77, Lat : 41.3906091, Lng : -73.95326349999999, Alt : 2.2600000000000002, Spd : 0.9170000553131104, GCrs : 224.25595092773438, VZ : -0.12300000339746475, Yaw : 0.0, U : 1}
'''        
