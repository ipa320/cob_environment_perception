#!/bin/bash

#scenes=(cupboard01 cupboard02 kitchen01 kitchen02 office01 office02 table01 table02)
scenes=(cupboard01)
svms=(/home/goa-sf/pcd_data/evaluations/set01/fpfh_training2/_svms/*.xml)
for f in ${scenes[*]}; do
    mkdir -p /home/goa-sf/pcd_data/evaluations/set01/results/$f/temp
    pcds=(/home/goa-sf/pcd_data/evaluations/set01/0_fpfh/$f/*.pcd)
    echo "process $f with ${#pcds[*]} pcd-files"
    parallel --progress --sshloginfile loginAll.txt "nice -19 /home/goa-sf/git/care-o-bot/cob_environment_perception_intern/cob_3d_mapping_features/bin/feature_evaluation_fpfh /home/goa-sf/pcd_data/evaluations/set01/normals/$f/mls_${f}_040rn.pcd {1} {2} > /home/goa-sf/pcd_data/evaluations/set01/results/$f/temp/{1/.}_{2/.}.csv" ::: ${pcds[*]} ::: ${svms[*]}

    temps=(/home/goa-sf/pcd_data/evaluations/set01/results/$f/temp/*.csv)
    echo -n "merge files: "
    ls /home/goa-sf/pcd_data/evaluations/set01/results/$f/temp/*.csv | wc -l
    for tmp in ${temps[*]}; do
      cat $tmp >> /home/goa-sf/pcd_data/evaluations/set01/results/$f/fpfh_${f}_res2.csv
    done
    gvfs-trash /home/goa-sf/pcd_data/evaluations/set01/results/$f/temp/
done

