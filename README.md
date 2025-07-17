## Dynamic Object Detection using PointPillar
simple program for detect 3D objects in oudoor using [torch implementation of PointPillar](https://github.com/zhulf0804/PointPillars) with ROS1 noetic
- input : Ouster Lidar pointcloud topic
- output : bounding boxes (by custom msg), rviz marker

## How to use
1. installation
```
cd catkin_ws/src
git clone
git submodule update --init --recursive

chmod +x src/preciction.py
```
2. build with catkin_make or build.
3. rosrun dynObjDet prediction.py
