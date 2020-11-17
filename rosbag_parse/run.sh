#!/bin/bash

dirname=$1
if [ ! -d $dirname ]; then
    mkdir $dirname
else
    rm -r $dirname
    mkdir $dirname
fi
cd $dirname

mkdir raw
cd raw
mkdir rgb depth
touch depth.txt rgb.txt

rosrun save_rgbd_from_k4a save_rgbd_from_k4a $dirname
