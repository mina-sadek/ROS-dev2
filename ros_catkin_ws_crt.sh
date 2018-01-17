#!/usr/bin/env sh

starting_dir="$PWD"
home_dir="$HOME"
echo home_dir:$home_dir

#1. If catkin workspace doesn't exist, Create a catkin workspace
if [ ! -d "$home_dir/ros_catkin_ws/src" ]; then
  # Control will enter here if $DIRECTORY doesn't exist.
  echo "Creating a catkin workspace @ $home_dir/ros_catkin_ws"
  cd ~
  mkdir -p $home_dir/ros_catkin_ws/src
  cd $home_dir/ros_catkin_ws/src
  echo "Initializing catkin workspace ..."
  catkin_init_workspace
else
  echo "ros_catkin_ws exists @ $home_dir/ros_catkin_ws"
fi

#2. Build the workspace
echo "Building catkin workspace ..."
cd $home_dir/ros_catkin_ws
catkin_make

