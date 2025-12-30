cmake_minimum_required(VERSION 3.0.2)
project(husky_mover)

## Catkin bağımlılıkları
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
)

## Catkin package tanımı
catkin_package()

## Python scriptini yüklenebilir yap
catkin_install_python(PROGRAMS
  src/husky_mover.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
