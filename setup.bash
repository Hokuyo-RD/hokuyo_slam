# export CMAKE_PREFIX_PATH=/opt/vtk8
# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/vtk8/lib
# export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/opt/pcl

#!/bin/bash
IFS_BACKUP=$IFS
IFS=$'\n'
i=0
for L in `cat data/ATC_area_2/init_pose.txt`
do
  i=`expr $i + 1`
  echo $L
done
IFS=$IFS_BACKUP