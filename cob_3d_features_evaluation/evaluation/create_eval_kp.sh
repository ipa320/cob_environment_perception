#!/bin/bash

RADIUS=0.7

DATE=`date +%d_%m_%g_%H_%M`
TMP="/tmp/blub"

for f in /media/TOSHIBA\ EXT/rosbag/freiburg_with_pc/*.bag
do
	echo "set bars 2.0">$TMP
	echo "set style fill empty">>$TMP
	echo "set terminal pdf">>$TMP
	echo 'set out "'$f'-kp.pdf"'>>$TMP
	echo "plot \\">>$TMP

	for RADIUS in 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0
	do
		echo "$f-$date-KP-radius-$RADIUS.csv" "$f-tf.csv" "$f-$date-eval-kp"
		evaluation/evaluate_kp.py "$f-$date-KP-radius-$RADIUS.csv" "$f-tf.csv" "$f-$date-eval-kp"
		echo '"'"$f-$date-eval-kp.csv"'" using 1:4 title "'$RADIUS'" with lines , \'>>$TMP
	done

	echo "1.0 with lines" >>$TMP
	gnuplot $TMP
done
