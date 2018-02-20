#!/bin/bash

mkdir dataset

wget http://www.ipb.uni-bonn.de/html/projects/changedetection2017/changedetection2017.zip -O dataset/changedetection2017.zip

unzip dataset/changedetection2017 -d dataset
rm dataset/changedetection2017.zip