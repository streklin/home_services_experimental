#!/bin/sh
xterm -e "source ~/catkin_ws/devel/setup.bash; roslaunch astra_launch astra.launch" &
sleep 5
xterm -e "source ~/catkin_ws/devel/setup.bash; roslaunch rover_platform marvin.launch" &
sleep 5
xterm -e "source ~/catkin_ws/devel/setup.bash; rosrun rover_platform_middleware middleware.js" &
sleep 5
xterm -e "source ~/catkin_ws/devel/setup.bash; roslaunch snowboy_ros snowboy.launch" &
