#!/bin/bash
source $HOME/projects/autoware/install/setup.bash
ros2 bag play $HOME/projects/resources/lsim/bag -r 0.2 -s sqlite3
