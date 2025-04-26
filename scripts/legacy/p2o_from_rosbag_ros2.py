#!/usr/bin/env python3
# Generate p2o from rosbag file (ROS2)

import sys
import os
import numpy as np
from rosbag2_py import Reader
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from pyproj import Transformer
import kgeom3d

# parameters

# information matrix (upper triangle) for odometry observations
odom_infom = '1e2 0 0 0 0 0 1e2 0 0 0 0 1e2 0 0 0 1e2 0 0 1e2 0 1e2'

def latlon_to_xyz(trans, lat, lon, alt):
    x, y = trans.transform(lat, lon)
    return x, y, alt

args = sys.argv
assert len(args) >= 5, "you must specify a rosbag file, lio topic, gnss topic, and gnss_cov_thre"

gnss_cov_thre = float(args[4])

# get path
filename = os.path.normpath(os.path.join(os.getcwd(), args[1]))
lio_topic_name = os.path.normpath(os.path.join(os.getcwd(), args[2]))
gnss_topic_name = os.path.normpath(os.path.join(os.getcwd(), args[3]))

# Sample convert to Japan Plane Rectangular Coordinate System No. 6
transformer = Transformer.from_crs("epsg:4326", 'epsg:6674')

# read the bag file
reader = Reader()
reader.open(filename)

type_map = reader.get_all_topics_and_types()
type_map = {topic_name: type_name for topic_name, type_name in type_map}

storage_filter = rosbag2_py.StorageFilter(topics=[lio_topic_name, gnss_topic_name])
reader.set_filter(storage_filter)

prev_gnss_t = 0
vertices = []
edges = []
np_poses = None
np_gnss_list = None
id = 0

while reader.has_next():
    serialized_message = reader.read_next()
    topic = serialized_message[0]
    msg_type = type_map[topic]
    msg = deserialize_message(serialized_message[1], msg_type)
    t_since_epoch = serialized_message[2].time_since_epoch * 1e-9

    if topic == lio_topic_name:
        if isinstance(msg, PoseStamped):
            id += 1
            pose = msg.pose
            np_pose = np.zeros((1, 8), dtype=np.float64)
            np_pose[0, 0] = t_since_epoch
            np_pose[0, 1] = pose.position.x
            np_pose[0, 2] = pose.position.y
            np_pose[0, 3] = pose.position.z
            np_pose[0, 4] = pose.orientation.x
            np_pose[0, 5] = pose.orientation.y
            np_pose[0, 6] = pose.orientation.z
            np_pose[0, 7] = pose.orientation.w
            if np_poses is None:
                np_poses = np_pose
            else:
                np_poses = np.append(np_poses, np_pose, axis=0)
            q = pose.orientation
            qstr = f'{q.x} {q.y} {q.z} {q.w}'
            vertices.append(f'VERTEX_SE3:QUAT {id} {pose.position.x} {pose.position.y} {pose.position.z} {qstr}')

    if topic == gnss_topic_name:
        if isinstance(msg, NavSatFix):
            if (msg.status.status == 0 or msg.status.status == 2) and id > 0:
                if msg.position_covariance[0] < gnss_cov_thre and (t_since_epoch - prev_gnss_t > 3):
                    x, y, z = latlon_to_xyz(transformer, msg.latitude, msg.longitude, msg.altitude)
                    np_gnss = np.zeros((1, 8), dtype=np.float64)
                    np_gnss[0, 0] = t_since_epoch
                    np_gnss[0, 1] = id
                    np_gnss[0, 2] = x
                    np_gnss[0, 3] = y
                    np_gnss[0, 4] = z
                    np_gnss[0, 5] = msg.position_covariance[0]
                    np_gnss[0, 6] = msg.position_covariance[4]
                    np_gnss[0, 7] = msg.position_covariance[8]
                    if np_gnss_list is None:
                        np_gnss_list = np_gnss
                    else:
                        np_gnss_list = np.append(np_gnss_list, np_gnss, axis=0)
                    prev_gnss_t = t_since_epoch

reader.close()

mean_gnss = np.mean(np_gnss_list, axis=0)
vertices.insert(0, f'VERTEX_SE3:QUAT 0 {mean_gnss[3]} {mean_gnss[2]} {mean_gnss[4]} 0 0 0 1')

for i in range(1, len(np_poses)):
    p = np_poses[i, :]
    prevp = np_poses[i - 1, :]
    dp = kgeom3d.ominus_se3(p[1:], prevp[1:])
    edges.append(f'EDGE_SE3:QUAT {i} {i + 1} {dp[0]} {dp[1]} {dp[2]} {dp[3]} {dp[4]} {dp[5]} {dp[6]} {odom_infom}')

for i in range(len(np_gnss_list)):
    id = int(np_gnss_list[i, 1])
    x = np_gnss_list[i, 2] - mean_gnss[2]
    y = np_gnss_list[i, 3] - mean_gnss[3]
    z = np_gnss_list[i, 4] - mean_gnss[4]
    xinfo = min(1.0, 1.0 / np_gnss_list[i, 5])
    yinfo = min(1.0, 1.0 / np_gnss_list[i, 6])
    zinfo = min(1.0, 1.0 / np_gnss_list[i, 7])
    gnss_infom = f'{xinfo} 0 0 {yinfo} 0 {zinfo}'
    edges.append(f'EDGE_LIN3D 0 {id} {y} {x} {z} {gnss_infom}')

for v in vertices:
    print(v)

for e in edges:
    print(e)
