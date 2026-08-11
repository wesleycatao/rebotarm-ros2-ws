cmake_minimum_required(VERSION 3.22)
project(rebotarm_moveit_config)

find_package(ament_cmake REQUIRED)

find_package(moveit_core QUIET)
find_package(moveit_ros_planning QUIET)
find_package(rclcpp QUIET)
find_package(rebotarm_msgs QUIET)
find_package(sensor_msgs QUIET)

if(
  moveit_core_FOUND
  AND moveit_ros_planning_FOUND
  AND rclcpp_FOUND
  AND rebotarm_msgs_FOUND
  AND sensor_msgs_FOUND
)
  add_executable(clearance_service_node src/clearance_service_node.cpp)
  target_compile_features(clearance_service_node PUBLIC cxx_std_17)
  ament_target_dependencies(
    clearance_service_node
    moveit_core
    moveit_ros_planning
    rclcpp
    rebotarm_msgs
    sensor_msgs
  )
  install(TARGETS clearance_service_node
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING
    "MoveIt C++ dependencies unavailable: clearance_service_node will not be built. "
    "MoveIt launch files will report this explicitly at runtime."
  )
endif()

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
