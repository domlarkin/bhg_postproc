#!/bin/bash
DATA_DIR='/home/user1/DATA/BHG'
echo $DATA_DIR
sudo chmod -R ugo=rX $DATA_DIR
echo 'Made directories executable and the data read only'

