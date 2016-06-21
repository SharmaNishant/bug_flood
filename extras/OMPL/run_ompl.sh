#!/bin/bash
#files=($(find ~/Desktop/path_planning -type f -name "source*"))
#for item in ${files[*]}
#do
#  echo $item | sed -e s/[^0-9]//g;
#done
base="/home/shivamthukral/Desktop/path_planning/"
outputFolderName="/OUTPUT_RRT_STAR"
cd /home/shivamthukral/ompl_ws/build_isolated/ompl/devel/bin
files=($(find $base -type d -name "*obstacles"))
for folder in ${files[*]}
do
   echo $folder 
   for i in {1..100}
    do
     sg_file="/source_goal_$i.txt"
     sg_file=$folder$sg_file
     echo $sg_file
     bm_file="/binary_data_$i.txt"
     bm_file=$folder$bm_file
     echo $bm_file
     output_file="/output_$i.txt"
     output_file=$folder$outputFolderName$output_file
     echo $output_file
     result_file="/result.txt"
     result_file=$folder$outputFolderName$result_file
     echo $result_file
     ./demo_OptimalPlanning -s $sg_file -b $bm_file -f $output_file -p RRTstar -r $result_file -o ThresholdPathLength
    done
done

