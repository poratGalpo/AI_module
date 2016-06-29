#!/bin/bash

xterm -e roscore &
sleep 5
xterm -e roslaunch freenect_launch freenect.launch depth_registration:=true &
sleep 10
xterm -e roslaunch rtabmap_ros rgbd_mapping.launch rtabmapviz:=false rtabmap_args:="--delete_db_on_start" &
sleep 8
#xterm -e cd /home/tomer/Desktop &
xterm -e python /home/osher/finalProject/AI_module/driver.py &
sleep 2
xterm -e python /home/osher/finalProject/AI_module/client_app.py &
rosrun rviz rviz -d /home/osher/finalProject/AI_module/rgbd.rviz &
