cmake_minimum_required(VERSION 3.8)
project(panda_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  # sensor_msgs
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(amra_star REQUIRED) #amra_star
find_package(my_robot_msgs REQUIRED)  

include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../amra_star/include  #  amra_star 
  ${my_robot_msgs_INCLUDE_DIRS}
)

# Executable of Topic demo
add_executable(target_pose_publisher src/target_pose_publisher.cpp)
target_link_libraries(target_pose_publisher ${EIGEN3_LIBRARIES})
ament_target_dependencies(target_pose_publisher rclcpp std_msgs geometry_msgs)

add_executable(pose_subscriber src/pose_subscriber.cpp)
target_link_libraries(pose_subscriber ${EIGEN3_LIBRARIES})
ament_target_dependencies(pose_subscriber rclcpp std_msgs geometry_msgs sensor_msgs tf2_ros pinocchio)

# Executable of Action with amra_star
# add_executable(move_to_pose_server src/move_to_pose_server.cpp)
# add_executable(amra_star_action_client src/amra_star_action_client.cpp)

# target_include_directories(move_to_pose_server 
# PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
# )
# target_include_directories(amra_star_action_client 
# PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
# )

# ament_target_dependencies(move_to_pose_server
#   rclcpp
#   rclcpp_action
#   geometry_msgs
#   pinocchio  
#   amra_star
#   my_robot_msgs
# )

# ament_target_dependencies(amra_star_action_client
#   rclcpp
#   rclcpp_action
#   geometry_msgs
#   pinocchio  
#   amra_star
#   my_robot_msgs
# )

# target_link_libraries(move_to_pose_server  ${EIGEN3_LIBRARIES})
# target_link_libraries(amra_star_action_client  ${EIGEN3_LIBRARIES})



#Execuatbale of move to pose

add_executable(pose_actuator_server src/pose_actuator_server.cpp)
add_executable(pose_actuator_client src/pose_actuator_client.cpp)
##
add_executable(test_my_action src/test_my_action.cpp)
##
target_include_directories(pose_actuator_server 
PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_include_directories(pose_actuator_client 
PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)
##
target_include_directories(test_my_action 
PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)
##
ament_target_dependencies(pose_actuator_server
  rclcpp
  rclcpp_action
  geometry_msgs
  pinocchio  
  my_robot_msgs
  sensor_msgs
  tf2_ros
)

ament_target_dependencies(pose_actuator_client
  rclcpp
  rclcpp_action
  geometry_msgs
  pinocchio  
  my_robot_msgs
  sensor_msgs
  tf2_ros
)

##
ament_target_dependencies(test_my_action
  rclcpp
  rclcpp_action
  geometry_msgs
  pinocchio  
  my_robot_msgs
  sensor_msgs
  tf2_ros
)
##
target_link_libraries(pose_actuator_server  ${EIGEN3_LIBRARIES})
target_link_libraries(pose_actuator_client  ${EIGEN3_LIBRARIES})

##
target_link_libraries(test_my_action  ${EIGEN3_LIBRARIES})
##

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(TARGETS
  target_pose_publisher
  pose_subscriber
  DESTINATION lib/${PROJECT_NAME})

#  
# install(TARGETS
#   move_to_pose_server
#   amra_star_action_client
#   DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  pose_actuator_server
  pose_actuator_client
  test_my_action ##
  DESTINATION lib/${PROJECT_NAME})

ament_package()
