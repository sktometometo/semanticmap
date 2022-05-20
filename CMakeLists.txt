cmake_minimum_required(VERSION 3.0.2)
project(semanticmap)

find_package(catkin REQUIRED
    message_generation
    roscpp
    rospy
    std_msgs
    geometry_msgs
    rviz
    )

catkin_python_setup()

add_message_files(
  FILES
  GridCell.msg
  GridCellArray.msg
  SemanticMapGrid.msg
  SemanticMapMetaData.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_generation roscpp rospy
)

catkin_install_python(PROGRAMS
    node_scripts/semantic_map_server.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
    src/semantic_map_grid.cpp
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_semanticmap.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
