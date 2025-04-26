#!/usr/bin/env python3

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import sys
import os
import numpy as np
import glob
from sensor_msgs_py import point_cloud2
import open3d as o3d

def find_db_file(bag_folder):
    """指定されたフォルダ内の最初の .db3 ファイルを検索"""
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

def extract_and_save_pointcloud(db_file, topic_name):
    conn, c = connect(db_file)
    msg_type_str = getMsgType(c, topic_name)
    if not msg_type_str:
        close(conn)
        return

    if msg_type_str != 'sensor_msgs/msg/PointCloud2':
        print(f"Error: Topic '{topic_name}' is not of type sensor_msgs/msg/PointCloud2. Found type: {msg_type_str}")
        close(conn)
        return

    timestamps, msgs_data = getAllMessagesInTopic(c, topic_name)
    count = 1
    print(f"Processing topic: {topic_name}")
    for msg_data in msgs_data:
        msg_type = get_message(msg_type_str)
        deserialized_msg = deserialize_message(msg_data, msg_type)

        # より効率的な点の抽出
        data = np.frombuffer(deserialized_msg.data, dtype=np.uint8).view(dtype=np.float32)
        if deserialized_msg.point_step != 0:
            points = data.reshape(-1, deserialized_msg.point_step // 4)[:, :3]
        else:
            points = np.array([])

        if points.size == 0:
            print(f"Warning: No valid points found in message at timestamp {timestamps[count-1]}. Skipping.")
            continue

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        o3d.io.write_point_cloud(f'cloud_{count:05}.pcd', pcd)
        count += 1
    close(conn)
    print(f"Saved {count - 1} point cloud files.")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python your_script_name.py <bag_folder> <pointcloud_topic_name>")
        sys.exit(1)

    bag_folder = os.path.normpath(os.path.join(os.getcwd(), sys.argv[1]))
    topic_name = sys.argv[2]

    db_file = find_db_file(bag_folder)
    if not db_file:
        print(f"Error: No .db3 file found in '{bag_folder}'.")
        sys.exit(1)

    extract_and_save_pointcloud(db_file, topic_name)