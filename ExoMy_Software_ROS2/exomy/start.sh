#!/bin/bash

# use:
# 'bash start webserver' to start the webserver.
# 'bash start websocket' to start the rosbridge websocket server.
# 'bash start rosnodes' to start the ROS2 nodes.
# 'bash start webrtc' to start the WebRTC signalling server and the local browser.
# 'bash all' to start all of the above.

function start_webserver()
{
	http-server --ssl --cert /home/ubuntu/exomy/ssl/server.crt --key /home/ubuntu/exomy/ssl/server.key /home/ubuntu/exomy/gui -p 55555 &
}

function start_websocket()
{
	. /opt/ros/galactic/setup.bash
    . /home/ubuntu/ros2_galactic/ros2-linux/setup.bash
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
}

function start_rosnodes()
{
	. /home/ubuntu/exomy/install/setup.bash
    . /home/ubuntu/ros2_galactic/ros2-linux/setup.bash
    ros2 launch /home/ubuntu/exomy/launch/exomy_pkg.launch.py &
}

function start_webrtc()
{
	cd /home/ubuntu/exomy/webrtc-web && node index.js &
    firefox https://localhost:8080 &  # Add -kiosk if desired.
}

function stop_all()
{
	sudo pkill -f http-server # To kill webserver.
	sudo pkill -f python3   # To kill websocket-server and ROS2 nodes.
	sudo pkill -f node      # To kill WebRTC signalling server.
	sudo pkill -f firefox   # To kill the local browser
}

if [[ $1 == "webserver" ]]
then
	start_webserver
elif [[ $1 == "websocket" ]]
then
    start_websocket
elif [[ $1 == "rosnodes" ]]
then
    start_rosnodes
elif [[ $1 == "webrtc" ]]
then
    start_webrtc
elif [[ $1 == "all" || -z $1 ]]  # Execute all if parameter is 'all' or if no parameter is given.
then
    start_webserver
	start_websocket
	start_rosnodes
	start_webrtc
elif [[ $1 == "stop" ]]
then
    stop_all
else
    echo "use one of the following parameters: webserver, websocket, rosnodes, webrtc, all, stop or no parameter (same as all)"
fi
