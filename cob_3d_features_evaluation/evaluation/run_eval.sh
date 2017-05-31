#!/bin/bash

RADIUS=0.6

DATE=`date +%d_%m_%g_%H_%M`
for f in /media/TOSHIBA\ EXT/rosbag/freiburg_with_pc/*bag
do
	echo "$f-$DATE-eval.csv"
	rm "$f-$date-eval.csv"
	rm "$f-tf.csv"
	bin/tf_from_bag "$f" "$f-tf.csv" /openni_camera /world
	bin/test_invariant_feature "$f" "$f-$date-eval.csv" $RADIUS
done
