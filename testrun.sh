#!/bin/sh
title1="roscore"
title2="opencr"

#cmd1="cd /etc"
#cmd2="cd ~/Documents"
#cmd3="cd /usr/local"
# echo "Please provide me with your SUDO password"
# read password 
# sudo sh -c 'echo 3000 > /sys/module/usbcore/parameters/usbfs_memory_mb' | printf '%s\n' '$password'  \

gnome-terminal --tab --title="$title1" -e "bash -c 'roscore; exec bash'" \
               --tab --title="$title2" -e "bash -c 'rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0; exec bash'" \
#gnome-terminal --tab --title="$title4" -e "bash -c 'sleep 10; roslaunch yolact_ros yolact_ros.launch; exec bash'"


#gnome-terminal & disown --working-directory=/home/coui/jieun/GitAhead/testinsertcloud3Dto2D/testpointcloud -x "bash -c 'cd /home/coui/jieun/Azure_ROS_wrapper/devel/; exec bash"