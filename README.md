## Dynamic Object Detection using PointPillar
simple program for detect 3D objects in oudoor using [torch implementation of PointPillar](https://github.com/zhulf0804/PointPillars) with ROS1 noetic
- input : Ouster Lidar pointcloud topic
- output : bounding boxes (by custom msg), rviz marker

## How to use
1. installation
```bash
cd catkin_ws/src
git clone https://github.com/KRISO-MIR-Lab/DL_based_dynObjDet.git

# -------------------------terminal in the container-----------------
cd DL_based_dynObjDet
conda create -n test python=3.7
conda activate test

cd PointPillars/
pip install -r requirements.txt
python setup.py build_ext --inplace
pip install .

cd dynObjDet
cd src

git clone https://github.com/zhulf0804/PointPillars.git

chmod +x src/prediction.py
```
2. build with catkin_make or build.
3. rosrun dynObjDet prediction.py

## When you use
<p align="center">
  <img src="https://github.com/user-attachments/assets/96f5e660-604c-4971-bad2-ad7eea174faf">
</p>
