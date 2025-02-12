#!/bin/bash

# 実行方法
# ./hokuyo_slam.bash <rosbagファイル> <ディレクトリ名>
# ディレクトリは data/に作られる。

if [ -z "$1" ]; then
  echo "Error: 引数が不足しています <フォルダ名>"
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
map_dir=$CURRENT;
echo rosbag dir: $rosbag_dir
echo 'ouput directory_name: '"$2"
echo 'rosbag file: ' "$1"
echo "All args are checked."

#------- config.csv 読み込み -------
if [ "$3" = "" ]; then
  options=(`cat config/config.csv`)
  echo option: $options
else
  options=(`cat $3`)
  echo option: $options
fi

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

# ディレクトリ作成
mkdir -p data/$2
mkdir -p data/$2/PCDs

# rosbag 移動
mv rosbag/$1 data/$2
# p2o
bash -c "python3 scripts/p2o_from_rosbag.py data/$2/$1 $lio_topic $gnss_topic > data/$2/output.p2o" # 引数2 input.bag
bash -c "./build/run_p2o data/$2/output.p2o"
#bash -c "gnuplot atc_odom_gnss.plt"

# p2o_fastlio_util
cd data/$2/PCDs 
bash -c "python3 ../../../scripts/extract_pcd.py ../$1 $pointcloud_topic" # ~/p2o_fastlio_util/extract_pcd 引数1 + 引数2

# p2o_fastlio_util におけるファイル整理
cd ./..
tail -n +2 output.p2o_out.txt > poses.txt
find . | grep pcd > clouds.txt
sort clouds.txt > sorted_clouds.txt
paste sorted_clouds.txt poses.txt > concat.txt
bash -c "./../../build/rearrange_pointcloud concat.txt $2"

# 絶対座標を相対座標に変換
cd ../..
bash -c "python3 scripts/pcd_to_Rcord.py data/$2/${2}_Acord.pcd data/$2/${2}_Rcord.pcd data/$2/output.p2o_out.txt data/$2/init_pose.txt"