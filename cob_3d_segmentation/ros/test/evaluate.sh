#!/bin/bash

cd `rospack find cob_3d_segmentation`
rm test/labeled/pc_Seg*
./bin/test_segmentation
./ros/test/toppm.sh
