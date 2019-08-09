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


###########
## Build ##
###########

add_library(${CLOUDWATCH_LOGGER_LIBRARY_TARGET}
        src/log_node.cpp
        src/log_node_param_helper.cpp
        )
ament_target_dependencies(${CLOUDWATCH_LOGGER_LIBRARY_TARGET}
        rclcpp
        rcl_interfaces
        std_msgs
        std_srvs
        aws_common
        aws_ros2_common
        AWSSDK
        cloudwatch_logs_common
        dataflow_lite
        file_management
        )
## Specify additional locations of header files
include_directories(
        include
        ${AWSSDK_INCLUDE_DIRS}
        )

## Declare a C++ executable
add_executable(${PROJECT_NAME}
        src/main.cpp
        )
target_link_libraries(${PROJECT_NAME}
        ${CLOUDWATCH_LOGGER_LIBRARY_TARGET}
        )


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
