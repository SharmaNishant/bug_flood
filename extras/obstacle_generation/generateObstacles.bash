#!/bin/bash

#gnome-terminal -e '/bin/bash -c "roscore" '

#mkdir ~/tmp/environment/

noObstacle=250
i=1
#generate 1000 enviornemnt at a temp location
/home/shivamthukral/ClionProjects/Generation_Bug_Flood/obstacleGeneration/Generation_Bug_Flood $noObstacle /home/shivamthukral/tmp/environment/

source /home/shivamthukral/ur_ws/devel/setup.bash

#run bugflood on those environments and keep the ones that are good
for ((j=1;j<=1000;j++))
do
	echo $j
	(timeout 10s rosrun bug_flood bug_flood /home/shivamthukral/tmp/environment/sg_$j.txt /home/shivamthukral/tmp/environment/map_$j.txt /home/shivamthukral/tmp/bugLog.txt )
	if [ "$?" -eq 134 ]; # ek hi baat hai isse replace kar de no no == hoga bhai haan ?? ?? abhi bhi ni hua na ho gaya bhai whatsapp
	then
		(echo "failed")
		continue
	else
		(echo "passed")
		#copy that enviornemnt to proper locatiopn	
		cp /home/shivamthukral/tmp/environment/sg_$j.txt /home/shivamthukral/tmp/obstacles/$noObstacle\_obstacles/sg_$i.txt
		cp /home/shivamthukral/tmp/environment/map_$j.txt /home/shivamthukral/tmp/obstacles/$noObstacle\_obstacles/map_$i.txt
		cp /home/shivamthukral/tmp/environment/image_$j.jpg /home/shivamthukral/tmp/obstacles/$noObstacle\_obstacles/image_$i.jpg
		i=`expr $i + 1`	

		if [ $i -gt 100 ]
		then
			break
		fi
	fi
	
done	 

