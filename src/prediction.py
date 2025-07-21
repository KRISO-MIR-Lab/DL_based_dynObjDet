#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import torch
import os

import functions.utils as utils
from dynObjDet.msg import DetectedObjectArray
from visualization_msgs.msg import MarkerArray

class PointPillarNode:
    def __init__(self):
        print("Node started")
        # 모델 초기화
        self.CLASSES = {
            'Pedestrian': 0, 
            'Cyclist': 1, 
            'Car': 2
            }
        
        self.is_cuda = torch.cuda.is_available()
        self.device = None
        if self.is_cuda==True:
            self.device = torch.device("cuda")
        else:
            self.device = torch.device("cpu")

        self.model_path = '/root/catkin_ws/src/dynObjDet/src/PointPillars/pretrained/epoch_160.pth'
        self.model = utils.load_pointpillars_model(self.device, self.CLASSES, self.model_path)       
        
        # ROS 설정
        rospy.init_node('dynObjDet_node')
        print("Node initialized")

        # self.sub = rospy.Subscriber('/ouster/points', PointCloud2, self.callback)
        self.sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.callback)
        self.pub = rospy.Publisher('/detected_objects', DetectedObjectArray, queue_size=10)
        self.vizPub = rospy.Publisher('pointpillar/markers', MarkerArray, queue_size=10)
    
    def callback(self, msg):
        print("Node - callback start")
        print("=== Point Cloud Debug Info ===")
        points = utils.pointcloud2_to_numpy(msg)
        print(f"Points shape: {points.shape}")
        print(f"Intensity range: {points[:, 3].min():.3f} ~ {points[:, 3].max():.3f}")
        print(f"Point count before filtering: {len(points)}")
        
        pc_torch = utils.preprocess_points(points)
        print(f"Point count after filtering: {len(pc_torch)}")
    
        if len(pc_torch) < 100:  # 너무 적은 포인트
            print("WARNING: Very few points after filtering!")

        # 1. PointCloud2 → numpy 변환
        points = utils.pointcloud2_to_numpy(msg)
        
        # 2. 모델 입력 형식으로 전처리
        pc_torch = utils.preprocess_points(points)
        print(f"pc_torch type: {type(pc_torch)}")

        # model_input = [pc_torch]
        model_input = pc_torch.to(self.device)
        print(f"model_input type: {type(model_input)}")

        # 3. 모델 추론
        results = None        
        print(f"before model predict")
        self.model.eval()
        with torch.no_grad():
            results = self.model(batched_pts=[model_input], mode='test')[0]

        print(f"result type: {type(results)}, {results}")
        # 4. 결과를 커스텀 메시지로 변환
        currTime = rospy.Time.now()
        detection_msg = utils.results_to_message(results, self.CLASSES, currTime, 'velodyne')
        vizMarker = utils.results_to_markers(results, self.CLASSES, currTime, 'velodyne')
        
        # 5. 발행
        
        self.pub.publish(detection_msg)
        self.vizPub.publish(vizMarker)
        print("Node - callback end")


if __name__ == '__main__':
    try:
        print("Program started") 
        predict = PointPillarNode()
        rospy.spin()
    except rospy. ROSInterruptException:
        print("Program exit")
        pass
