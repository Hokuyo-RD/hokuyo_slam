# Copyright (C) 2024 Kiyoshi Irie
# MIT License

import sys
import numpy as np
from rosbag2_py import Reader
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
from builtin_interfaces.msg import Time

def extract_and_save_pointcloud_ros2(rosbag_path, topic_name):
    reader = Reader()
    reader.open(rosbag_path)

    type_map = reader.get_all_topics_and_types()
    type_map = {topic_name: type_name for topic_name, type_name in type_map}

    storage_filter = rosbag2_py.StorageFilter(topics=[topic_name])
    reader.set_filter(storage_filter)

    count = 1
    print(topic_name)

    while reader.has_next():
        serialized_message = reader.read_next()
        msg_type = type_map[serialized_message[0]]
        msg = deserialize_message(serialized_message[1], msg_type)

        if isinstance(msg, PointCloud2):
            points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
            if points.size == 0:
                print('error.')
                reader.close()
                exit(0)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            o3d.io.write_point_cloud(f'cloud_{count:05}.pcd', pcd)
            count += 1

    reader.close()

extract_and_save_pointcloud_ros2(sys.argv[1], sys.argv[2])
