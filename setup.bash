# export CMAKE_PREFIX_PATH=/opt/vtk8
# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/vtk8/lib
# export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/opt/pcl

#!/bin/bash
posefile="data/ATC_area_2/init_pose.txt"

while IFS=, read -r init_pose1 init_pose2 init_pose3 init_pose4 init_pose5 init_pose6 init_pose7
do
 echo "${init_pose1} ${init_pose2} ${init_pose3} ${init_pose3} ${init_pose4} ${init_pose5} ${init_pose6} ${init_pose7}"
done < ${posefile}