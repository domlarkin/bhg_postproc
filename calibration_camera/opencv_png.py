#!/usr/bin/env python3

import cv2

in_file = '/home/user1/21122026calibration/left-0016.png'

in_file = '/home/user1/Data/20220123_222532_139/BOSON16_SN_160171/BOSON160171_16_20220123_222535_541.png'
original = cv2.imread(in_file, -1)
print('original.shape:\n',original.dtype,original.size,original.shape)
cv2.imshow('undistort',original)
cv2.waitKey(0)

