#!/usr/bin/env python3

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import sys
import os
import csv
import numpy as np
import glob

args = sys.argv

# get path
bag_folder = os.path.normpath(os.path.join(os.getcwd(), args[1]))
output_csv = os.path.normpath(os.path.join(os.getcwd(), args[2]))
topic_name = args[3]
thre = float(args[4])

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

if __name__ == "__main__":
    db_file = find_db_file(bag_folder)
    if not db_file:
        print(f"Error: No .db3 file found in '{bag_folder}'.")
        exit()

    conn, c = connect(db_file)

    msg_type_str = getMsgType(c, topic_name)
    if not msg_type_str:
        close(conn)
        exit()
    msg_type = get_message(msg_type_str)

    timestamps, msgs_data = getAllMessagesInTopic(c, topic_name)
    num_messages = len(msgs_data)

    # NumPy 配列を事前に確保
    covariances = np.empty((num_messages, 9))

    for i, msg_data in enumerate(msgs_data):
        deserialized_msg = deserialize_message(msg_data, msg_type)
        covariances[i] = deserialized_msg.position_covariance

    # NumPy の機能を使って fix 率を計算
    north_fixed = covariances[:, 0] <= thre
    east_fixed = covariances[:, 4] <= thre
    vertical_fixed = covariances[:, 8] <= thre

    avg_north_fix_rate = np.mean(north_fixed) * 100 if north_fixed.size > 0 else 0
    avg_east_fix_rate = np.mean(east_fixed) * 100 if east_fixed.size > 0 else 0
    avg_vertical_fix_rate = np.mean(vertical_fixed) * 100 if vertical_fixed.size > 0 else 0

    # NumPy の機能を使ってばらつきの平均を計算
    avg_north_variance = np.mean(covariances[:, 0]) if covariances.size > 0 else 0
    avg_east_variance = np.mean(covariances[:, 4]) if covariances.size > 0 else 0
    avg_vertical_variance = np.mean(covariances[:, 8]) if covariances.size > 0 else 0

    with open(output_csv, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)

        # Write fix rates
        csv_writer.writerow(['北方向のfix率[%]', '東方向のfix率[%]', '鉛直方向のfix率[%]'])
        csv_writer.writerow([f'{avg_north_fix_rate:.10f}', f'{avg_east_fix_rate:.10f}', f'{avg_vertical_fix_rate:.10f}'])

        # Write average variances
        csv_writer.writerow(['北方向のばらつきの平均[m]', '東方向のばらつきの平均[m]', '鉛直方向のばらつきの平均[m]'])
        csv_writer.writerow([f'{avg_north_variance:.10f}', f'{avg_east_variance:.10f}', f'{avg_vertical_variance:.10f}'])

        # Write individual variances
        csv_writer.writerow(['北方向のばらつき[m]', '東方向のばらつき[m]', '鉛直方向のばらつき[m]'])
        for n_var, e_var, v_var in covariances[:, [0, 4, 8]]:
            csv_writer.writerow([f'{n_var:.10f}', f'{e_var:.10f}', f'{v_var:.10f}'])

    close(conn)
    print(f"Data written to {output_csv}")