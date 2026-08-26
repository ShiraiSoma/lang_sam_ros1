cmake_minimum_required(VERSION 3.5)
project(lang_sam_person_following)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  lang_sam_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs lang_sam_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lang_sam_person_following_node
  src/lang_sam_person_following_node.cpp
  src/main.cpp
)

add_dependencies(lang_sam_person_following_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(lang_sam_person_following_node
  ${catkin_LIBRARIES}
)

install(TARGETS lang_sam_person_following_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
