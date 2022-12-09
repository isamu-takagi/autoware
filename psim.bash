#!/bin/bash
source $HOME/projects/autoware/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit map_path:=$HOME/projects/resources/psim/map $@
