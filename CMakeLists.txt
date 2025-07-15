cmake_minimum_required(VERSION 3.10)
project(panda_moveit_config)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_ros_planning_interface
  geometry_msgs
  )

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(moncube src/moncube.cpp)
add_dependencies(moncube ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#add_executable(pick_place_demo src/pick_place_demo.cpp)
install(TARGETS moncube
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#target_link_libraries(pick_place_demo ${catkin_LIBRARIES})
target_link_libraries(moncube ${catkin_LIBRARIES})



install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
