#! /usr/bin/env python3
import os
import cv2 as cv
import yaml
import numpy as np
import exiftool
#from calibration_camera.utils import load_coefficients

def load_coefficients(path):
    '''Loads camera matrix and distortion coefficients.'''
    # FILE_STORAGE_READ
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode('K').mat()
    dist_matrix = cv_file.getNode('D').mat()

    cv_file.release()
    return [camera_matrix, dist_matrix]

#top_dirs = ['/media/user1/T7/Week1_Data/20220119_META','/media/user1/T7/Week1_Data/20220120_META']
top_dirs = ['/media/user1/T7/Week1_Data/20220120_META']
calibration_file_dir = '/home/user1/catkin_ws/src/usma_bhg/resources'
sn_list = ['18285141','21122026','124812','160171',]

for top_dir in top_dirs:
    #top_dir = '/media/user1/T7/20220120_META'
    for dirName, subdirList, fileList in os.walk(top_dir):
        print("")
        for fname in fileList:
            full_filename = os.path.join(dirName,fname)
            if fname.startswith('BOSON') and fname.endswith('png'):
                cam_sn = fname.split('_')[0][-6:]
            elif fname.startswith('BLACK') and fname.endswith('png'):
                cam_sn = fname.split('_')[0][-8:] 
            else:
                continue
            calib_file = calibration_file_dir + '/' + cam_sn + 'calibrationv3.yaml'
            print("=== Using the calibration file %s for the file:" % calib_file)

            mtx, dist = load_coefficients(calib_file)
            original = cv.imread(full_filename, -1)
            img_show = cv.resize(original, (640, 480))
            cv.putText(img_show,fname[-23:-4],(20,120), cv.FONT_HERSHEY_SIMPLEX, .7, (251,251,251), 1)                            
            cv.imshow('original',img_show)
            dst = cv.undistort(original, mtx, dist, None, None)
            img_show2 = cv.resize(dst, (640, 480))
            cv.putText(img_show2,fname[-23:-4],(20,120), cv.FONT_HERSHEY_SIMPLEX, .7, (251,251,251), 1)
            cv.imshow('undistort',img_show2)
            cv.waitKey(1)
            date_var = full_filename.split('/')[4]
            filename_out = full_filename.replace(date_var,'Done',1)        
            print("=== === %s" %filename_out)
            pathname = os.path.dirname(filename_out)
            # Check whether the specified path exists or not
            if not os.path.exists(pathname):          
                # Create a new directory because it does not exist 
                os.makedirs(pathname)
                print("The new directory is created!")        
            #print(filename_out)           
            cv.imwrite(filename_out,dst)
            
            # copy metadat over to new file
            with exiftool.ExifTool() as et:
                et.execute(b"-tagsFromFile",  bytes(full_filename, 'utf-8') , bytes(filename_out, 'utf-8'))
            rm_orig_filename = filename_out + '_original'
            if os.path.exists(rm_orig_filename):
                os.remove(rm_orig_filename)

cv.destroyAllWindows()


        
