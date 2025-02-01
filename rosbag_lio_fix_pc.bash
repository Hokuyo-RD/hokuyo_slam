#!/bin/bash

#------- 環境変数の確認 -------
echo ros workspace: ${ROS_WORKSPACE:?ROS_WORKSPACE is Undefined}

#------- 引数の確認 -------
# 第1引数
if [ -z "$1" ]; then
  echo "Error: 引数が不足しています <arg1>"
  exit 1
fi
# 第2引数
if [ -z "$2" ]; then
  echo "Error: 引数が不足しています <arg2>"
  exit 1
fi

# ファイルが存在するかチェック
FILE=rosbag/$1
if [ ! -f "$FILE" ]; then
  echo "Error: File $FILE does not exist."
  exit 1
else
  echo "rosbag file: $FILE exists."
fi

#------- カレントディレクトリの取得 -------
CURRENT=$(cd $(dirname $0);pwd)
echo current dir: $CURRENT
rosbag_dir=$CURRENT/rosbag;
echo rosbag dir: $rosbag_dir
echo "All args are checked."

#------- config.csv 読み込み -------
options=(`cat config/config.csv`)

for i in ${!options[@]}; do
 if [ $i -gt 0 ]; then
  j=$((${i}-1))
  option_arr[$j]=`echo ${options[$i]} | cut -d ',' -f 2`
  fi
done

gnss_topic="${option_arr[0]}";
pointcloud_topic="${option_arr[1]}";
lio_topic="${option_arr[2]}";

sleep 3

source /opt/ros/$ROS_DISTRO/setup.bash
source $ROS_WORKSPACE/devel/setup.bash

gnome-terminal --tab -t "Tab 0" -- bash -c "roscore; bash"
sleep 2
gnome-terminal --tab -t "hokuyo_lio" -- bash -c "roslaunch hokuyo_lio hokuyo_lio_node_with_yaml.launch; bash"
gnome-terminal --tab -t "rosbag play" -- bash -c "cd ${rosbag_dir}; rosbag play $1; bash"
gnome-terminal --tab -t "sync_lio_pc" -- bash -c "echo sync_lio_pc working!!; rosrun sync_lio_pc sync_lio_pc; bash"
gnome-terminal --tab -t "rosbag record" -- bash -c "cd ${rosbag_dir}; echo timeout $((`rosbag info ${rosbag_dir}/$1 | grep -i "duration" | awk '{ print $3 }' | sed -e 's/[^0-9]//g'`)) rosbag record -O $2 $gnss_topic $pointcloud_topic $lio_topic; timeout $((`rosbag info ${rosbag_dir}/$1 | grep -i "duration" | awk '{ print $3 }' | sed -e 's/[^0-9]//g'`-10)) rosbag record -O $2 $gnss_topic $pointcloud_topic $lio_topic; bash"