#!/bin/bash

RADII=8
ANGLES=32

DATE="19_02_15_10_32"

SAVEIFS=$IFS
IFS=$(echo -en "\n\b")

for V in 2 4 8 16 32 64 128 256
do

for f in /media/TOSHIBA\ EXT/rosbag/freiburg_with_pc/*bag
do
	FILE=`ls "$f-timing-$V-$ANGLES-"*$DATE-eval.csv`
	evaluation/create_eval_timing_single_file.sh "$f" "$FILE"

	FILE=`ls "$f-timing-$RADII-$V-"*$DATE-eval.csv`
	evaluation/create_eval_timing_single_file.sh "$f" "$FILE"

done
done

IFS=$SAVEIFS
