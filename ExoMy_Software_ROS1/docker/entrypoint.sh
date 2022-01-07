#!/bin/bash

# ReneB: We use SSH to execute commands on the Raspberry Pi host from this container.
# Copy the private part of the SSH key pair to /root for authentication so SSH can be used without password by a Python script.
# It is assumed that the public half of the SSH key pair is already installed on the Raspberry Pi host.
# The chmod is needed to give only user permissions otherwise authentication will not work.
# See also https://upcloud.com/community/tutorials/use-ssh-keys-authentication
cp -r /root/exomy_ws/src/exomy/.ssh /root
chmod -R 700 /root/.ssh

if [[ $1 == "config" ]]
then
	cd /root/exomy_ws/src/exomy/scripts
	bash
elif [[ $1 == "autostart" ]]
then
	source /opt/ros/melodic/setup.bash
	cd /root/exomy_ws
	catkin_make
	http-server --ssl --cert /root/exomy_ws/src/exomy/ssl/server.crt --key /root/exomy_ws/src/exomy/ssl/server.key src/exomy/gui -p 55555 &

	source devel/setup.bash
	roslaunch exomy exomy.launch

	bash
elif [[ $1 == "devel" ]]
then
	cd /root/exomy_ws
	source /opt/ros/melodic/setup.bash
	# catkin_make
	# source devel/setup.bash
	bash
else
	bash
fi

