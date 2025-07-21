#!/usr/bin/env python3

import os
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
import rosbag
import rospy

rospy.init_node('kitti2bag_writer', anonymous=True)

print(f"start")
velodyne_dir = '/root/KITTI/testing/velodyne'
bag = rosbag.Bag('kitti_testing_velodyne.bag', 'w')
files = sorted(os.listdir(velodyne_dir))
frame_id = 'velodyne'
start = rospy.Time.now()

for i, fname in enumerate(files):
    print(f"processing {i}th frame")
    points = np.fromfile(os.path.join(velodyne_dir, fname), dtype=np.float32).reshape(-1, 4)
    header = Header()
    header.stamp = start + rospy.Duration(0.1 * i)
    header.frame_id = frame_id
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1)
    ]
    pc2_msg = pc2.create_cloud(header, fields, points)
    bag.write('/velodyne_points', pc2_msg, header.stamp)
bag.close()

print(f"[✓] Finished writing {len(files)} frames to {output_bag}")