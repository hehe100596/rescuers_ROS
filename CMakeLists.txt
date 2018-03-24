cmake_minimum_required(VERSION 2.8.3)
project(xbacov04)

find_package(catkin REQUIRED COMPONENTS
  roslint
  art_msgs
  cv_bridge
  geometry_msgs
  image_geometry
  rospy
  std_msgs
  roslaunch
  rostest
  art_utils
  qt_build
)

find_package(Qt5 COMPONENTS Core Gui Widgets Qml Quick REQUIRED)

catkin_python_setup()

set(ROSLINT_PYTHON_OPTS "--max-line-length=250")
roslint_python()
roslint_add_test()

catkin_package(
  CATKIN_DEPENDS geometry_msgs art_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Qml_INCLUDE_DIRS}
  ${Qt5Quick_INCLUDE_DIRS}
)

file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.qrc)

#install(DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
#
#catkin_install_python(PROGRAMS src/app.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#
#catkin_install_python(PROGRAMS src/projector_node.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
