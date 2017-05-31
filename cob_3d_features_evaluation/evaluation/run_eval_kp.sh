#!/bin/bash

DATE=`date +%d_%m_%g_%H_%M`
TMP="/tmp/blub"

for f in /media/TOSHIBA\ EXT/rosbag/freiburg_with_pc/*.bag
do

	for RADIUS in 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0
	do
		echo "$f-$date-KP-radius-$RADIUS.csv"
		rm "$f-$date-KP-radius-$RADIUS.csv"
		bin/test_invariant_feature "$f" "$f-$date-KP-radius-$RADIUS.csv" $RADIUS
	done

done
