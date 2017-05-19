#!/bin/bash

RADIUS=0.7

DATE=`date +%d_%m_%g_%H_%M`

for f in /media/josh/90488461488447C4/ft_eval/evaluation_19_02_rad07/*bag
do

	evaluation/evaluate_pr.py "$f-$date-eval.csv" "$f-tf.csv" "$f-$date-eval-feature-$RADIUS"


done

