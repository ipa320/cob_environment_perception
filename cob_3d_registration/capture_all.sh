#!/bin/bash

states=("state_trans_x" "state_trans_y" "state_rot_z" "state_rot_x")
vals=("true" "false")

for a in ${vals[@]}
do
	for b in ${vals[*]}
do
	for c in ${vals[*]}
do
	for d in ${vals[*]}
do

add="_"
if [ $a == "true" ];then
add=$add"tx"
fi
if [ $b == "true" ];then
add=$add"ty"
fi
if [ $c == "true" ];then
add=$add"rz"
fi
if [ $d == "true" ];then
add=$add"rx"
fi


pre="/home/goa-jh/dat3/$add/"
mkdir $pre

cat head>ros/launch/temp.launch
echo '<param name="prefix" value='\"$pre'" />'>>ros/launch/temp.launch
echo '<param name="state_trans_x" value='\"$a'" />'>>ros/launch/temp.launch
echo '<param name="state_trans_y" value='\"$b'" />'>>ros/launch/temp.launch
echo '<param name="state_rot_z" value='\"$c'" />'>>ros/launch/temp.launch
echo '<param name="state_rot_x" value='\"$d'" />'>>ros/launch/temp.launch
cat tail>>ros/launch/temp.launch

#start
roslaunch registration temp.launch
#cat ros/launch/temp.launch

done
done
done
done

