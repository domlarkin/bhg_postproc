#! /usr/bin/env python3

import os
import shutil

empty = []
top_dir = '/media/user1/T7' 

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

remove_empty_files(top_dir)
dirs_deleted = delete_empty_folders(top_dir)
print("Empty Directories:")
for item in dirs_deleted:
    print(item)


