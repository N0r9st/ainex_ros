#!/usr/bin/env zsh
sleep 10
export ROS_IP=192.168.149.1
export ROS_MASTER_URI=http://192.168.149.1:11311
export ROS_HOSTNAME=$ROS_IP

export working_space=/home/ubuntu

if [ $ZSH_VERSION ]; then
  . /opt/ros/noetic/setup.zsh
  . $working_space/ros_ws/devel/setup.zsh
elif [ $BASH_VERSION ]; then
  . /opt/ros/noetic/setup.bash
  . $working_space/ros_ws/devel/setup.bash
else
  . /opt/ros/noetic/setup.sh
  . $working_space/ros_ws/devel/setup.sh
fi

#export DISPLAY=:0.0
exec "$@"
