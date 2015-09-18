#!/bin/bash

TR="0.5"
RT="0.35"

for NUM in 10 100
do
	DIR="result/run_${TR}_${RT}_${NUM}"
	./run_tests.py result/office_odom.bag $TR $RT $NUM
	mkdir $DIR
	mv result/eval_em_* $DIR

	echo "generating summary..."
	echo "" > $DIR/summary.csv
	for F in `ls $DIR/*csv`
	do
		echo -n "$F;"  >> $DIR/summary.csv
		./eval_result.py $F >> $DIR/summary.csv
	done
	
	mkdir ${DIR}_maps/
	mv $DIR/*map ${DIR}_maps/
done
