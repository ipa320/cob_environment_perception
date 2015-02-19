#!/bin/bash

RADII=8
ANGLES=32

for V in 2 4 8 16 32 64 128 256
do

for f in /media/TOSHIBA\ EXT/rosbag/freiburg_with_pc/*bag
do
	FILE=`ls "$f-timing-$V-$ANGLES-"*eval.csv`
	evaluation/create_eval_timing_single_file.sh "$f" "$FILE"

	FILE=`ls "$f-timing-$RADII-$V-"*eval.csv`
	evaluation/create_eval_timing_single_file.sh "$f" "$FILE"

done
done
