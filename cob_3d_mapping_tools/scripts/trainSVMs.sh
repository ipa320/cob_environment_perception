#!/bin/bash


# $1: output folder, $2: input features, $3: input labels
runSVM(){
    rosrun cob_3d_mapping_features fpfh_svm_trainer_2 -f $1 -k 10
}

runReduce(){
    rosrun cob_3d_mapping_tools reduce_fpfh_training_data $1 $2 -k 500
}

#ls /home/goa-sf/pcd_data/evaluations/set01/fpfh_training/050rnmls_050*/fpfh_sph.pcd | parallel --sshloginfile loginAll.txt 'nice -19 /home/goa-sf/git/care-o-bot/cob_environment_perception_intern/cob_3d_mapping_tools/bin/reduce_fpfh_training_data {} {//}/reduced/{/} -k 500'

ls -d /home/goa-sf/pcd_data/evaluations/set01/fpfh_training2/*/reduced/ | parallel --sshloginfile loginAll.txt 'nice -20 /home/goa-sf/git/care-o-bot/cob_environment_perception_intern/cob_3d_mapping_features/bin/fpfh_svm_trainer -f {} -k 10'

