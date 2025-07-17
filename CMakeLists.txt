cmake_minimum_required(VERSION 3.0.2)
project(dynObjDet)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  message_generation
)

## 메시지 파일 정의
add_message_files(
  FILES
  DetectedObject.msg
  DetectedObjectArray.msg
)

## 메시지 빌드
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## catkin 패키지 정의
catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs visualization_msgs message_runtime
)

## 실행 가능한 Python 스크립트 설치 (src 디렉토리 기준)


install(PROGRAMS
  src/prediction.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## src 하위의 모든 py 파일 포함 (utils 등)
install(DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
  FILES_MATCHING PATTERN "*.py"
)