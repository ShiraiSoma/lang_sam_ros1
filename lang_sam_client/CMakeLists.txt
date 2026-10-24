cmake_minimum_required(VERSION 3.5)
project(lang_sam_client)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/lang_sam_client_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
