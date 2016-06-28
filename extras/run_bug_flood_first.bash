#!/bin/bash


planner="bug_flood_first";

base="/home/nishant/catkin_ws/src/bug_flood/obstacles/"
outputFolderName="/result"

source /home/nishant/catkin_ws/devel_isolated/setup.bash

files=($(find $base -type d -name "*obstacles"))
for folder in ${files[*]}
do
   echo $folder 
   for i in {1..100}
    do
     sg_file="/sg_$i.txt"
     sg_file=$folder$sg_file
     echo $sg_file
     bm_file="/map_$i.txt"
     bm_file=$folder$bm_file
     echo $bm_file
     result_file="/result_"$planner".txt"
     result_file=$folder$outputFolderName$result_file
     echo $result_file

	 rosrun bug_flood bug_flood $sg_file $bm_file $result_file first
    done
done

