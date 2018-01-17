#!/usr/bin/env sh

starting_dir="$PWD"
home_dir="$HOME"
echo home_dir:$home_dir

sudo apt-get install wmctrl
sudo apt-get install imagemagick

if [ ! -d "$home_dir/ros_catkin_ws/src" ]; then
  echo "ros_catkin_ws doesn't exit"
  if [! "./ros_catkin_ws_crt.sh"]; then
    echo "please download \"ros_catkin_ws_crt.sh\" file first to continue"
    echo "exiting"
    return
  else
    ./ros_catkin_ws_crt.sh
  fi
else
  echo "ros_catkin_ws exists @ $home_dir/ros_catkin_ws"
fi

#2. move to the workspace
#echo "Building catkin workspace ..."
#cd $home_dir/ros_catkin_ws/src


if [ -d "$home_dir/ros_catkin_ws/src/ros_001" ]; then
  echo "old ros_001 exists"
  echo "compressing old ros_001 for backup ..."
  cd $home_dir/ros_catkin_ws/src
  #rm ros_001_bkup_*
  f_name="ros_001_bkup_$(date '+%y%m%d_%H%M%S').tar.gz"
  tar -czvf "$f_name" ros_001
  echo "old ros_001 is backed up in $f_name"
  echo "deleting old ros_001"
  rm -rf ros_001
  cd $starting_dir
fi

# download and extract the ros_001 folder into the ros_catkin_ws/src
echo "Downloading ros_001 into $home_dir/ros_catkin_ws/src"
#tar -xvzf ros_001.tar.gz -C $home_dir/catkin_ws/src
cd $home_dir/ros_catkin_ws/src
git clone https://github.com/mina-sadek/ROS-dev2.git

mv ./ROS-dev2/ros_catkin_ws/ros_001 ./ros_001
rm -r ./ROS-dev2

# Rebuild the workspace
echo "rebuilding the ros_catkin_ws workspace"
cd $home_dir/ros_catkin_ws
catkin_make

echo "Done installing ros_001 at location: $home_dir/ros_catkin_ws/src/ros_001"
echo "===================================="
echo "To run the system:"
echo "Open a shell and source the setup.bash of your workspace by running this command:"
echo "source ~/ros_catkin_ws/devel/setup.bash"
echo "Then, run the diffBot system by running the launcher using the next command:"
echo "roslaunch ros_001 diffBot_launch.launch"




