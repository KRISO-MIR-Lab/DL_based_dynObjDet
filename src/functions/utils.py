#!/usr/bin/env python

import rospy
import numpy as np
import torch

from sensor_msgs import point_cloud2 as pc2
from functions.utils import *
from PointPillars import *
from dynObjDet.msg import DetectedObject, DetectedObjectArray
from PointPillars.pointpillars.model import PointPillars
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler



def pointcloud2_to_numpy(msg):
        '''
        as in function name.
        '''
        rawData = pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
        points = np.array(list(rawData), dtype=np.float32)
        
        return points

def preprocess_points(points):
    '''
    preprocess points(npy array) into model input 
    '''
    # 좌표 범위 필터링 (예: KITTI 기준)
    x_range = [0, 70]
    y_range = [-40, 40]
    z_range = [-3, 1]
    
    mask = (
        (points[:, 0] >= x_range[0]) & (points[:, 0] <= x_range[1]) &
        (points[:, 1] >= y_range[0]) & (points[:, 1] <= y_range[1]) &
        (points[:, 2] >= z_range[0]) & (points[:, 2] <= z_range[1])
    )
    
    filtered_points = points[mask]
    
    # 모델별 전처리 (예: zhulf0804/PointPillars 기준)
    return torch.from_numpy(filtered_points).float()

def load_pointpillars_model(device, classes, model_path):
    model = PointPillars(nclasses=len(classes)).to(device)
    model.load_state_dict(torch.load(model_path))

    return model

def results_to_message(results, classes, time, frame='os_sensor'):
    '''
    transform model ouput -> cusmtom msg
    '''
    detection_array = DetectedObjectArray()
    detection_array.header.stamp = time
    detection_array.header.frame_id = frame
    
    # 결과 파싱 (모델마다 다름)
    pred_boxes = results['lidar_bboxes']      # (N, 7): [x, y, z, dx, dy, dz, yaw]
    pred_scores = results['scores']    # (N,): confidence scores
    pred_labels = results['labels']    # (N,): class ids
    
    for i in range(len(pred_boxes)):
        if pred_scores[i] > 0.5:  # 임계값 필터링
            detected_obj = DetectedObject()
            
            # 기본 정보
            detected_obj.id = int(i)
            detected_obj.class_id = int(pred_labels[i])
            detected_obj.confidence = float(pred_scores[i])
            
            # 3D 바운딩 박스 정보
            bbox = pred_boxes[i]
            detected_obj.center.x = bbox[0]
            detected_obj.center.y = bbox[1]
            detected_obj.center.z = bbox[2]
            detected_obj.size.x = bbox[3]    # dx
            detected_obj.size.y = bbox[4]    # dy
            detected_obj.size.z = bbox[5]    # dz
            detected_obj.orientation = bbox[6]  # yaw!!!!! reference: Z-, CCW
            
            detection_array.objects.append(detected_obj)
    
    return detection_array

def results_to_markers(results, classes, time, frame='os_sensor'):
    marker_array = MarkerArray()

    pred_boxes = results['lidar_bboxes']      # (N, 7): [x, y, z, dx, dy, dz, yaw]
    pred_scores = results['scores']    # (N,): confidence scores
    pred_labels = results['labels']    # (N,): class ids

    for i in range(len(pred_boxes)):
        if pred_scores[i] < 0.5:  # 임계값 필터링
            continue

        box = pred_boxes[i]
        score = float(pred_scores[i])
        label_id = int(pred_labels[i])
        label_name = list(classes.keys())[list(classes.values()).index(label_id)]
        
        marker = Marker()
        marker.header.stamp = time
        marker.header.frame_id = frame

        marker.ns = "pointPillar_bbox"
        marker.id = i
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = box[0]
        marker.pose.position.y = box[1]
        marker.pose.position.z = box[2]

        q = quaternion_from_euler(0, 0, box[6])
        marker.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        marker.scale.x = box[3]
        marker.scale.y = box[4]
        marker.scale.z = box[5]
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        marker.lifetime = rospy.Duration(0.5)
        marker_array.markers.append(marker)

        text_marker=Marker()
        text_marker.header.stamp = time
        text_marker.header.frame_id = frame
        text_marker.ns = "pointPillar_label"
        text_marker.id = 1000 + i
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = box[0]
        text_marker.pose.position.y = box[1]
        text_marker.pose.position.z = box[2] + box[5]/2 + 0.2
        text_marker.scale.z = 0.6
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = f"{label_name} ({score:.2f})"
        text_marker.lifetime = rospy.Duration(0.5)
        marker_array.markers.append(text_marker)

    return marker_array