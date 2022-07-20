# iris_drone
#Iris Drone Simulation - ROS Melodic - Ubuntu 18.04
#!/bin/bash
echo "Iris_drone setup"
read -p "Press enter to continue"


# From https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux

echo "Cloning ArduPilot repository"
sleep 1

git clone https://github.com/peppegti/ardupilot.git
cd ardupilot
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

echo 'export PATH=$PATH:$HOME/ardupilot/Tools/autotest' >> ~/.bashrc 
echo 'export PATH=/usr/lib/ccache:$PATH' >> ~/.bashrc 

~/.bashrc



# From https://ardupilot.org/dev/docs/using-gazebo-simulator-with-sitl.html

sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update

sudo apt install gazebo9 libgazebo9-dev

sudo apt upgrade libignition-math2
sudo apt-get update

# check if "gazebo --verbose" works


echo "Plugin installation"
sleep 1

cd
git clone https://github.com/peppegti/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install

# check if "gazebo --verbose worlds/iris_arducopter_runway.world" works
# add on .bashrc file >>  alias killg='killall gzclient && killall gzserver && killall rosmaster'



# Upgrade gazebo ros
sudo apt update & sudo apt install ros-melodic-gazebo-ros*
sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras

wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh

sudo apt-get install ros-melodic-rqt ros-melodic-rqt-common-plugins ros-melodic-rqt-robot-plugins

echo 'export ROS_HOSTNAME=localhost'  >> ~/.bashrc
echo 'export ROS_MASTER_URI=http://localhost:11311'  >> ~/.bashrc


#From https://ardupilot.org/mavproxy/docs/getting_started/download_and_installation.html#linux


# for python 2
sudo apt-get install python-dev python-opencv python-wxgtk4.0 python-pip python-matplotlib python-lxml python-pygame
pip install PyYAML mavproxy --user
echo "export PATH=$PATH:$HOME/.local/bin" >> ~/.bashrc

# sudo usermod -a -G dialout <username>   TODO

pip install mavproxy pymavlink --user --upgrade



# Download Iris_Drone
cd
mkdir catkin_ws
cd catkin_ws
mkdir src
catkin_make
cd catkin_ws/src
git clone https://github.com/peppegti/iris_drone.git
cd ..
catkin_make
source devel/setup.bash
rospack profile

# Run simulation
roslaunch iris_drone simulation_setup_iris_only.launch 

