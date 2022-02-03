#!/usr/bin/env python3

from utils import load_coefficients
import cv2

calib_file = '/home/user1/catkin_ws/src/usma_bhg/resources/160171calibrationv3.yaml'
in_file = '/home/user1/21122026calibration/left-0016.png'

in_file = '/home/user1/Data/20220123_222532_139/BOSON16_SN_160171/BOSON160171_16_20220123_222535_541.png'

# Load coefficients
mtx, dist = load_coefficients(calib_file)
original = cv2.imread(in_file, -1)
print('original.shape:\n',original.dtype,original.size,original.shape)


dst = cv2.undistort(original, mtx, dist, None, None)
print('dst.shape:\n',dst.dtype,dst.size,dst.shape)

cv2.imwrite('/home/user1/undist.png',dst)
