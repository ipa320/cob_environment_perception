#!/bin/bash

for file in $( find `rospack find cob_3d_segmentation`/test/labeled/*.pcd )
do
rosrun cob_3d_mapping_tools pcd_to_ppm --in $file --out $file.ppm
done
