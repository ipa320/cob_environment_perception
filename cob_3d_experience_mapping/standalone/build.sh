#!/bin/bash

SYSTEMS=( "Linux" "Android" )

for SYSTEM in "${SYSTEMS[@]}"
do
	cmake -D CMAKE_SYSTEM_NAME2=$SYSTEM .
	make
done
