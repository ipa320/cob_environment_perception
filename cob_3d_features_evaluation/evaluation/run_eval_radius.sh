#!/bin/bash


DATE=`date +%d_%m_%g_%H_%M`
for RADIUS in 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0
do
	for f in /media/TOSHIBA\ EXT/rosbag/freiburg_with_pc/*bag
	do
		echo "$f-$DATE-eval-radius-$RADIUS.csv"
		rm "$f-$date-eval-radius-$RADIUS.csv"
		bin/test_invariant_feature "$f" "$f-$date-eval-radius-$RADIUS.csv" $RADIUS
	done
done
