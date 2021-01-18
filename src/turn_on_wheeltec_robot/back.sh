#!/bin/sh
pkill -f send_mark1.py &  #杀死进程
sleep 1
python ./send_mark1.py &  #启动进程
sleep 1
rostopic pub -1 /clicked_point geometry_msgs/PointStamped  '{header: {frame_id: 'map'},point: {x: 0,y: 0,z: 0}}'  #回0号点
exit
