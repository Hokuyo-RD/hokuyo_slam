#!/usr/bin/env python3

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import sys
import os
import numpy as np
import glob
import kgeom3d
from pyproj import Transformer

# parameters
odom_infom = '1e2 0 0 0 0 0 1e2 0 0 0 0 1e2 0 0 0 1e2 0 0 1e2 0 1e2'

def find_db_file(bag_folder):
    db_files = glob.glob(os.path.join(bag_folder, '*.db3'))
    if db_files:
        return db_files[0]
    else:
        return None

def connect(sqlite_file):
    conn = sqlite3.connect(sqlite_file)
    c = conn.cursor()
    return conn, c

def close(conn):
    conn.close()

def getAllElements(cursor, table_name, print_out=False):
    cursor.execute('SELECT * from({})'.format(table_name))
    records = cursor.fetchall()
    if print_out:
        print("\nAll elements:")
        for row in records:
            print(row)
    return records

def isTopic(cursor, topic_name, print_out=False):
    boolIsTopic = False
    topicFound = []
    records = getAllElements(cursor, 'topics', print_out=False)
    for row in records:
        if(row[1] == topic_name):
            boolIsTopic = True
            topicFound = row
    if print_out:
        if boolIsTopic:
            print('\nTopic named', topicFound[1], ' exists at id ', topicFound[0] ,'\n')
        else:
            print('\nTopic', topic_name ,'could not be found. \n')
    return topicFound

def getAllMessagesInTopic(cursor, topic_name, print_out=False):
    timestamps = []
    messages = []
    topicFound = isTopic(cursor, topic_name, print_out=False)
    if not topicFound:
        print('Topic', topic_name ,'could not be found. \n')
    else:
        cursor.execute('SELECT timestamp, data FROM messages WHERE topic_id = ?', (topicFound[0],))
        records = cursor.fetchall()
        for row in records:
            timestamps.append(row[0])
            messages.append(row[1])
    return timestamps, messages

def getMsgType(cursor, topic_name, print_out=False):
    msg_type = None
    cursor.execute('SELECT type FROM topics WHERE name = ?', (topic_name,))
    result = cursor.fetchone()
    if result:
        msg_type = result[0]
        if print_out:
            print(f'\nMessage type in {topic_name} is {msg_type}')
    else:
        print(f'\nTopic {topic_name} not found.')
    return msg_type

def latlon_to_xyz(trans, lat, lon, alt):
    x, y = trans.transform(lat, lon)
    return x, y, alt

if __name__ == "__main__":
    args = sys.argv
    assert len(args) >= 5, "Usage: ros2 run your_package_name your_script_name <bag_folder> <lio_topic> <gnss_topic> <gnss_cov_threshold>"

    bag_folder = os.path.normpath(os.path.join(os.getcwd(), args[1]))
    lio_topic_name = args[2]
    gnss_topic_name = args[3]
    gnss_cov_thre = float(args[4])

    db_file = find_db_file(bag_folder)
    if not db_file:
        print(f"Error: No .db3 file found in '{bag_folder}'.")
        exit()

    conn, c = connect(db_file)

    lio_msg_type_str = getMsgType(c, lio_topic_name)
    gnss_msg_type_str = getMsgType(c, gnss_topic_name)

    if not lio_msg_type_str or not gnss_msg_type_str:
        close(conn)
        exit()

    lio_timestamps, lio_msgs_data = getAllMessagesInTopic(c, lio_topic_name)
    gnss_timestamps, gnss_msgs_data = getAllMessagesInTopic(c, gnss_topic_name)

    lio_msg_type = get_message(lio_msg_type_str)
    gnss_msg_type = get_message(gnss_msg_type_str)

    num_lio = len(lio_timestamps)
    vertices = [None] * (num_lio + 1)  # Pre-allocate list for vertices
    edges = []
    np_poses_list = [None] * num_lio
    id_counter = 0

    # Sample convert to Japan Plane Rectangular Coordinate System No. 6
    transformer = Transformer.from_crs("epsg:4326", 'epsg:6674')

    # Process LIO data
    for i in range(num_lio):
        timestamp = lio_timestamps[i] * 1e-9
        deserialized_msg = deserialize_message(lio_msgs_data[i], lio_msg_type)
        pose = deserialized_msg.pose.pose

        id_counter += 1
        np_poses_list[i] = np.array([timestamp,
                                      pose.position.x, pose.position.y, pose.position.z,
                                      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

        q = pose.orientation
        qstr = f'{q.x} {q.y} {q.z} {q.w}'
        vertices[id_counter] = f'VERTEX_SE3:QUAT {id_counter} {pose.position.x} {pose.position.y} {pose.position.z} {qstr}'

        if i > 0:
            prev_pose_np = np_poses_list[i-1][1:]
            current_pose_np = np_poses_list[i][1:]
            dp = kgeom3d.ominus_se3(current_pose_np, prev_pose_np)
            edges.append(f'EDGE_SE3:QUAT {id_counter-1} {id_counter} {dp[0]} {dp[1]} {dp[2]} {dp[3]} {dp[4]} {dp[5]} {dp[6]} {odom_infom}')

    np_poses = np.array(np_poses_list)

    # Process GNSS data
    valid_gnss_data = []
    for timestamp, msg_data in zip(gnss_timestamps, gnss_msgs_data):
        deserialized_msg = deserialize_message(msg_data, gnss_msg_type)
        if (deserialized_msg.status.status == 0 or deserialized_msg.status.status == 2) and deserialized_msg.position_covariance[0] < gnss_cov_thre:
            valid_gnss_data.append((timestamp * 1e-9, deserialized_msg))

    mean_gnss = np.zeros(3)
    if valid_gnss_data:
        gnss_positions = np.array([latlon_to_xyz(transformer, msg.latitude, msg.longitude, msg.altitude)
                                   for _, msg in valid_gnss_data])
        mean_gnss = np.mean(gnss_positions, axis=0)
        vertices[0] = f'VERTEX_SE3:QUAT 0 {mean_gnss[0]} {mean_gnss[1]} {mean_gnss[2]} 0 0 0 1'

        for timestamp, msg in valid_gnss_data:
            gnss_xyz = latlon_to_xyz(transformer, msg.latitude, msg.longitude, msg.altitude)
            x = gnss_xyz[0] - mean_gnss[0]
            y = gnss_xyz[1] - mean_gnss[1]
            z = gnss_xyz[2] - mean_gnss[2]
            xinfo = min(1.0, 1.0 / msg.position_covariance[0]) if msg.position_covariance[0] > 0 else 1.0
            yinfo = min(1.0, 1.0 / msg.position_covariance[4]) if msg.position_covariance[4] > 0 else 1.0
            zinfo = min(1.0, 1.0 / msg.position_covariance[8]) if msg.position_covariance[8] > 0 else 1.0
            gnss_infom = f'{xinfo} 0 0 {yinfo} 0 {zinfo}'

            closest_lio_id = 0
            min_diff = float('inf')
            gnss_t = timestamp
            for j, lio_t in enumerate(np_poses[:, 0]):
                diff = abs(lio_t - gnss_t)
                if diff < min_diff:
                    min_diff = diff
                    closest_lio_id = j + 1

            if closest_lio_id > 0 and closest_lio_id <= id_counter:
                edges.append(f'EDGE_LIN3D 0 {closest_lio_id} {y} {x} {z} {gnss_infom}')
                edges.append(f'EDGE_LLA 0 {closest_lio_id} {msg.latitude} {msg.longitude} {msg.altitude}')

    close(conn)

    for v in vertices:
        if v is not None:
            print(v)

    for e in edges:
        print(e)