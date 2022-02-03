#!/usr/bin/env python3

from chessboard import calibrate_chessboard
from utils import load_coefficients, save_coefficients

# Parameters
IMAGES_FORMAT = '.png'
SQUARE_SIZE = 2.8
WIDTH = 5
HEIGHT = 7

'''
IMAGES_DIR = '/home/user1/124812calibration'

print(IMAGES_DIR)
# Calibrate 
ret, mtx, dist, rvecs, tvecs = calibrate_chessboard(
    IMAGES_DIR, 
    IMAGES_FORMAT, 
    SQUARE_SIZE, 
    WIDTH, 
    HEIGHT
)
# Save coefficients into a file
save_coefficients(mtx, dist, "/home/user1/124812calibrationv3.yaml")

#=======================================================================================
IMAGES_DIR = '/home/user1/160171calibration'
print(IMAGES_DIR)
# Calibrate 
ret, mtx, dist, rvecs, tvecs = calibrate_chessboard(
    IMAGES_DIR, 
    IMAGES_FORMAT, 
    SQUARE_SIZE, 
    WIDTH, 
    HEIGHT
)
# Save coefficients into a file
save_coefficients(mtx, dist, "/home/user1/160171calibrationv3.yaml")
'''
#=======================================================================================
#IMAGES_DIR = '/home/user1/18285141calibration'
IMAGES_DIR = '/home/user1/deleteme/flir_calibration_18285141'

print(IMAGES_DIR)
# Calibrate 
ret, mtx, dist, rvecs, tvecs = calibrate_chessboard(
    IMAGES_DIR, 
    IMAGES_FORMAT, 
    SQUARE_SIZE, 
    WIDTH, 
    HEIGHT
)
# Save coefficients into a file
save_coefficients(mtx, dist, "/home/user1/18285141calibrationv3.yaml")
'''
#=======================================================================================
IMAGES_DIR = '/home/user1/21122026calibration'
print(IMAGES_DIR)

# Calibrate 
ret, mtx, dist, rvecs, tvecs = calibrate_chessboard(
    IMAGES_DIR, 
    IMAGES_FORMAT, 
    SQUARE_SIZE, 
    WIDTH, 
    HEIGHT
)
# Save coefficients into a file
save_coefficients(mtx, dist, "/home/user1/21122026calibrationv3.yaml")

#=======================================================================================
'''
# https://medium.com/vacatronics/3-ways-to-calibrate-your-camera-using-opencv-and-python-395528a51615
