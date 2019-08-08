cmake_minimum_required(VERSION 3.5)
project(cloudwatch_logger)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(aws_common REQUIRED)
find_package(aws_ros2_common REQUIRED)
find_package(cloudwatch_logs_common REQUIRED)
find_package(dataflow_lite REQUIRED)
find_package(file_management REQUIRED)

set(CLOUDWATCH_LOGGER_LIBRARY_TARGET ${PROJECT_NAME}_lib)

#link_directories("${AWSSDK_LIB_DIR}")

###########
## Build ##
###########

add_library(${CLOUDWATCH_LOGGER_LIBRARY_TARGET}
        src/log_node.cpp
        src/log_node_param_helper.cpp
        )

## Specify additional locations of header files
include_directories(
        include
        ${rclcpp_INCLUDE_DIRS}
        ${rcl_interfaces_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
        ${std_srvs_INCLUDE_DIRS}
        ${aws_common_INCLUDE_DIRS}
        ${aws_ros2_common_INCLUDE_DIRS}
        ${AWSSDK_INCLUDE_DIRS}
        ${cloudwatch_logs_common_INCLUDE_DIRS}
        ${dataflow_lite_INCLUDE_DIRS}
        ${file_management_INCLUDE_DIRS}
        )
ament_target_dependencies(${CLOUDWATCH_LOGGER_LIBRARY_TARGET}
        rclcpp
        rosidl_typesupport_cpp
        aws_common
        aws_ros2_common
        cloudwatch_logs_common
        )
target_link_libraries(${CLOUDWATCH_LOGGER_LIBRARY_TARGET}
        ${rclcpp_LIBRARIES}
        ${rcl_interfaces_LIBRARIES}
        ${std_msgs_LIBRARIES}
        ${std_srvs_LIBRARIES}
        ${aws_common_LIBRARIES}
        ${aws_ros2_common_LIBRARIES}
        ${cloudwatch_logs_common_LIBRARIES}
        ${dataflow_lite_LIBRARIES}
        ${file_management_LIBRARIES}
        )

## Declare a C++ executable
add_executable(${PROJECT_NAME} src/main.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${CLOUDWATCH_LOGGER_LIBRARY_TARGET} ${rclcpp_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} ${CLOUDWATCH_LOGGER_LIBRARY_TARGET})


#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME} ${CLOUDWATCH_LOGGER_LIBRARY_TARGET}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(
        DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h")
install(
        DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config)
install(
        DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)
ament_export_include_directories(include)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rcl_interfaces)
ament_export_dependencies(rclcpp)
ament_export_dependencies(aws_common)
ament_export_dependencies(aws_ros2_common)
ament_export_dependencies(cloudwatch_logs_common)
ament_export_dependencies(dataflow_lite)
ament_export_dependencies(file_management)

ament_package()
