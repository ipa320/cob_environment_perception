#!/bin/bash

#roscore &
#rosrun rviz rviz &
roslaunch cob_3d_mapping_demonstrator startup.launch &
rosrun rviz rviz
