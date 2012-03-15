#!/bin/bash

# $1: output folder, $2: point density, $3: noise, [$4: shape radius size]
generatePlane(){
  rosrun cob_3d_mapping_tools fpfh_primitives -o $1 -s $2 -g $3 -P 
}

generateEdge(){
  rosrun cob_3d_mapping_tools fpfh_primitives -o $1 -s $2 -g $3 -E
}

generateCorner(){
  rosrun cob_3d_mapping_tools fpfh_primitives -o $1 -s $2 -g $3 -C
}

generateCyl(){
  rosrun cob_3d_mapping_tools fpfh_primitives -o $1 -s $2 -g $3 -r $4 -Z 
}

generateSph(){
  rosrun cob_3d_mapping_tools fpfh_primitives -o $1 -s $2 -g $3 -r $4 -S
}

#mkdir -p set01/fpfh_training_synthetic/corners
rng=(0.0005 0.001 0.002)
s=(0.006)
for g in ${rng[*]}; do
  echo -n "$g "
  #for s in ${steps[*]}; do
    #echo $s
    #generateCorner set01/fpfh_training_synthetic/corners/ $s $g 
  for r in $(seq 0.025 0.005 0.11); do
    echo $r
    generateCyl set01/fpfh_training_synthetic/cylinders/ $s $g $r
  done
done
