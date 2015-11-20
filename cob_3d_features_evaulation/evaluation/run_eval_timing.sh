#!/bin/bash

RADIUS=0.7

DATE=`date +%d_%m_%g_%H_%M`

RADII=8
ANGLES=32

for V in 2 4 8 16 32 64 128 256
do
for f in /media/TOSHIBA\ EXT/rosbag/freiburg_with_pc/*bag
do
	echo "$f-timing-$V-$ANGLES-$DATE-eval.csv"
	rm "$f-timing-$V-$ANGLES-$DATE-eval.csv"
	rm "$f-tf.csv"
	bin/tf_from_bag "$f" "$f-tf.csv" /openni_camera /world
	bin/test_invariant_feature "$f" "$f-timing-$V-$ANGLES-$DATE-eval.csv" $RADIUS $V $ANGLES
done
done

for V in 2 4 8 16 32 64 128 256
do
for f in /media/TOSHIBA\ EXT/rosbag/freiburg_with_pc/*bag
do
	echo "$f-timing-$RADII-$V-$DATE-eval.csv"
	rm "$f-timing-$RADII-$V-$DATE-eval.csv"
	rm "$f-tf.csv"
	bin/tf_from_bag "$f" "$f-tf.csv" /openni_camera /world
	bin/test_invariant_feature "$f" "$f-timing-$RADII-$V-$DATE-eval.csv" $RADIUS $RADII $V
done
done
