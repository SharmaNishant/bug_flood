#!/bin/bash


planner="FMTstar";

base="/home/nishant/catkin_ws/src/bug_flood/obstacles/"
outputFolderName="/result"

cd /home/nishant/catkin_ws/build_isolated/ompl/devel/bin

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
     output_file="/tmp/output.txt"
     echo $output_file
     result_file="/result_"$planner".txt"
     result_file=$folder$outputFolderName$result_file
     echo $result_file
     ./demo_OptimalPlanning -s $sg_file -b $bm_file -f $output_file -p $planner -r $result_file -o ThresholdPathLength
    done
done

