#!/bin/bash

#sim_vehicle.py -v ArduCopter -f gazebo-iris --console

x-terminal-emulator -e "sim_vehicle.py -v ArduCopter -f gazebo-iris --console -M --map"
#--map to run with the map


exit 0
