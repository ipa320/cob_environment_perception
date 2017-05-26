#!/bin/bash

RADIUS=0.7

DATE=`date +%d_%m_%g_%H_%M`
TMP="/tmp/blub"

for f in /media/TOSHIBA\ EXT/rosbag/freiburg_with_pc/*bag
do
	echo "set bars 2.0">$TMP
	echo "set style fill empty">>$TMP
	echo "set terminal pdf">>$TMP
	echo 'set out "'$f'-feature.pdf"'>>$TMP
	echo "plot \\">>$TMP

	echo "$f-$DATE-eval.csv"
	evaluation/evaluate.py "$f-$date-eval.csv" "$f-tf.csv" "$f-$date-eval-feature-$RADIUS"

	FILES=`ls "$f-$date-eval-feature-$RADIUS"*csv`
	SAVEIFS=$IFS
	IFS=$(echo -en "\n\b")
	for ff in $FILES
	do
		echo $ff
		echo '"'"$ff"'" using 1:4 title "'${ff##*_}'" with lines , \'>>$TMP
	done
	IFS=$SAVEIFS

	echo "1.0 with lines" >>$TMP
	gnuplot $TMP

	echo "set bars 2.0" >$TMP
	echo "set style fill empty" >>$TMP
	echo "set terminal pdf">>$TMP
	echo 'set out "'$f'-timing.pdf"'>>$TMP
	echo 'plot "'"$f-$date-eval-feature-$RADIUS""_timing.csv"'" using 1:3:2:6:5:xticlabels(1) with candlesticks title '"'Execution time' whiskerbars, ''  using 1:4:4:4:4 with candlesticks lt -1 notitle, '' using 1:4 with linespoints lt 3 pt 13 notitle" >>$TMP
	gnuplot $TMP

done

