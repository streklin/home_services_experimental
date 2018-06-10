#!/bin/sh
xterm -e "source ~/catkin_ws/devel/setup.bash; roslaunch rover_platform marvin_simulation.launch" &
sleep 5
xterm -e "source ~/catkin_ws/devel/setup.bash; rosrun rover_platform_middleware middleware.js" &
sleep 5
xterm -e "source ~/catkin_ws/devel/setup.bash; roslaunch rover_platform ai.launch" &
sleep 5
xterm -e "source ~/catkin_ws/devel/setup.bash; rosrun rover_platform navigator" &
sleep 5
xterm -e "source ~/catkin_ws/devel/setup.bash; rosrun rover_platform remoteControl" &
sleep 5
xterm -e "source ~/catkin_ws/devel/setup.bash; roslaunch snowboy_ros snowboy.launch" &
sleep 5
xterm -e "source ~/catkin_ws/devel/setup.bash; rosrun rover_platform aws-node.py"
