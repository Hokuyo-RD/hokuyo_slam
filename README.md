# hokuyo_slam

このパッケージは、hokuyo_lioの軌跡をRTK-GNSSの情報を使って補正し、
補正した軌跡に沿って、YVTの3Dスキャン点群を並べて、
緯度・軽度による絶対座標の情報を付与した3D点群地図を作成します。

# ros2 対応
04/27 sqlite3 形式の ROS2のROSBAG への対応完了。
p2o_gnsslog_from_rosbag_ros2.py, p2o_from_rosbag_ros2.py, extract_pcd_ros2.py, pcd_to_Rcord.py

04/27 mcap 形式のROS2のROSBAG への対応対応
04/27 mcap 形式と sqlite3 形式の両方を1つのコードで対応
(両方のrosbagがある場合はmcapを優先する。処理が早いから)
## dependency
```
# numpy
pip3 install numpy==1.24.4 (version >=1.17.3 <1.25.0)

# mcap

/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
# setting path

brew install mcap
```

## 使用したパッケージ
- p2o: https://github.com/furo-org/p2o
- p2o_fastlio_util: https://github.com/kiyoshiiriemon/p2o_fastlio_util

これらのソースコードを改変して使用しました。実行用のbashファイルを北陽電機 髙橋が作成しました。

## Requirements
Ubuntu 20.04 ROS1 Noetic

This package is dependent on Eigen3, C++14, and pcl 1.14

```bash
# Proj
sudo apt-get install libsqlite3-dev sqlite3
wget https://download.osgeo.org/proj/proj-9.4.1.tar.gz
tar -zxvf proj-9.4.1.tar.gz
cd proj-9.4.1
mkdir build
cd build
cmake ..
cmake --build .
sudo cmake --build . --target install

# for vtk
sudo apt-get install libeigen3-dev
sudo apt-get -y install qtbase5-dev
sudo apt-get -y install clang
sudo apt-get -y install qtcreator
sudo apt-get -y install libqt5x11extras5-dev

# pcl 1.14
cd ~/github
wget https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.14.1/source.tar.gz -O pcl.tar.gz
tar -xvf pcl.tar.gz
cd pcl
cmake -Bbuild -DCMAKE_INSTALL_PREFIX=/opt/pcl .
cmake --build build
sudo cmake --install build
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/opt/pcl
```
hokuyo_lio and sync_lio
```bash
# 04/27 時点で未対応
# hokuyo_lio
# cd ~/catkin_ws/src
# git clone https://github.com/Hokuyo-RD/hokuyo_lio.git
# catkin build hokuyo_lio

# 04/27 時点で未対応
# sync_lio_pc
# cd ~/catkin_ws/src
# git clone https://github.com/Hokuyo-RD/sync_lio_pc.git
# catkin build sync_lio_pc

# hokuyo_slam
cd ~/github
git clone https://github.com/Hokuyo-RD/hokuyo_slam.git
cd hokuyo_slam

export CMAKE_PREFIX_PATH=/opt/vtk8
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/vtk8/lib
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/opt/pcl

mkdir build
cmake -Bbuild . && cmake --build build
```

python extract
```
sudo apt-get install python3-pip
pip3 install tqdm
pip3 install open3d

```
未対応：環境変数の設定
```
# ~/.bashrc に一行追記
# export ROS_WORKSPACE=$HOME/catkin_ws
# or export ROS_WORKSPACE=$HOME/<your_workingspace>
```

## Usage

1. locate bag file under the rosbag/
2. Edit csv file ROSTOPIC name for p2o use.
```bash
# config/config.csv
オプション,指定値,デフォルト値
gnss_topic,/fix,/fix,
pointcloud_topic,/hokuyo3d3/hokuyo_cloud2,/hokuyo3d3/hokuyo_cloud2,
lio_topic,/hokuyo_lio/sync_odom,/hokuyo_lio/sync_odom,
```
3. Get ROSBAG for p2o 
```bash
# if you already have <lio_fix_pointcloud-rosbag>, skip this step.
./get_rosbag.bash <input-rosbag> <lio_fix_pointcloud-rosbag>
```
4. Run p2o and utility
```bash
./setup.bash
./hokuyo_slam.bash <lio_fix_pointcloud-rosbag> <output-directory>
```
