#!/bin/bash


# $1: output folder, $2: input features, $3: input labels
runReduce(){
    rosrun cob_3d_mapping_tools reduce_fpfh_training_data $1 $2 -k 500
}

if [[ "$#" != 2 || "$1" < 0 ]] ; then
  exit 0
fi
  
sets=(fpfh_training2/*)
echo ${#sets[*]}
i="$1"
while [ "$i" -lt "$2" ] ; do
  mkdir -p ${sets["$i"]}/reduced
  #echo "${sets["$i"]}/reduced"
  files=(${sets["$i"]}/*.pcd)
  for pcd in ${files[@]}; do
    if [[ $pcd =~ (fpfh_[a-zA-Z0-9]*.pcd) ]]; then
      echo "$pcd"
      runReduce "$pcd" ${sets["$i"]}/reduced/${BASH_REMATCH[1]}
    fi
  done
  let i++
done
