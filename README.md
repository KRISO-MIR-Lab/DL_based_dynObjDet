## Dynamic Object Detection using PointPillar
simple program for detect 3D objects in oudoor using [torch implementation of PointPillar](https://github.com/zhulf0804/PointPillars) with ROS1 noetic
- input : Ouster Lidar pointcloud topic
- output : bounding boxes (by custom msg), rviz marker

## How to use
1. installation
```bash
cd catkin_ws/src
git clone https://github.com/PyoSH/dynObjDet.git

cd dynObjDet
cd src

git clone https://github.com/zhulf0804/PointPillars.git

chmod +x src/prediction.py
```
2. build with catkin_make or build.
3. rosrun dynObjDet prediction.py

## When you use
<p align='center'>
  <img src="https://github.com/user-attachments/assets/96f5e660-604c-4971-bad2-ad7eea174faf">
</p>
