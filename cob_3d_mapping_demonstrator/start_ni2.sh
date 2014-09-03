#!/bin/bash

#roscore &
#rosrun rviz rviz &
roslaunch cob_3d_mapping_demonstrator startup_ni2.launch &
rosrun rviz rviz
