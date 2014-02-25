#!/bin/bash


# $1: output folder, $2: input features, $3: input labels
runExtract(){
    rosrun cob_3d_mapping_tools extract_feature_values $1 \
  -f $2 -f $3 \ # -f $4 -f $5\
	-l normals/kitchen_old/mls_kitchen_old_030rn.pcd \
	#-l normals/table01/mls_table01_020rn.pcd \
	#-l normals/table02/mls_table02_020rn.pcd \
	-l normals/cupboard_old/mls_cupboard_old_030rn.pcd
}

#kit=(0_fpfh/kitchen01/*.pcd)
#tab1=(0_fpfh/table01/*.pcd)
#tab2=(0_fpfh/table02/*.pcd)
#cup1=(0_fpfh/cupboard01/*.pcd)
kit=(0_fpfh/kitchen_old/*.pcd)
cup=(0_fpfh/cupboard_old/*.pcd)

if [[ ${#cup[*]} -eq ${#kit[*]} ]] 
  then
  i=0
  for file in ${kit[@]} ; do
    if [[ $file =~ ([0-9]{3}(rn|rnmls)_[0-9]{3}rf) ]]
      then
	      folder=${BASH_REMATCH[1]}
        mkdir -p fpfh_training_old/$folder
        echo $folder
        runExtract fpfh_training_old/$folder/ ${kit[i]} ${cup[i]}
    fi
    let i++
  done
fi
