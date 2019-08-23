cmake_minimum_required(VERSION 3.5)
project(cloudwatch_logger)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Enable strict compiler flags if possible.
include(CheckCXXCompilerFlag)
# Removed -Wmissing-declarations until gmock is ignored
# removed -Werror=pedantic until ros.h is fixed
set(FLAGS -Wno-long-long -Wall -Wextra -Wcast-align -Wcast-qual -Wformat -Wwrite-strings)
foreach(FLAG ${FLAGS})
  check_cxx_compiler_flag(${FLAG} R${FLAG})
  if(${R${FLAG}})
    set(WARNING_CXX_FLAGS "${WARNING_CXX_FLAGS} ${FLAG}")
  endif()
endforeach()

if(NOT DEFINED CXX_DISABLE_WERROR)
  set(WARNING_CXX_FLAGS "-Werror ${WARNING_CXX_FLAGS}")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${WARNING_CXX_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
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
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        )
install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        )
install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
        )
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
        )

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


#############
## Tests ##
#############

ament_add_gmock(test_log_node
        test/log_node_test.cpp
        )
target_include_directories(test_log_node PRIVATE
        include
        ${cloudwatch_logs_common_INCLUDE_DIRS}
        ${dataflow_lite_INCLUDE_DIRS}
        ${file_management_INCLUDE_DIRS}
        ${AWSSDK_INCLUDE_DIRS}
        ${aws_common_INCLUDE_DIRS}
        ${rclcpp_INCLUDE_DIRS}
        )
target_link_libraries(test_log_node
        ${CLOUDWATCH_LOGGER_LIBRARY_TARGET}
        )

ament_add_gmock(test_log_node_param_helper
        test/log_node_param_helper_test.cpp
        )
target_include_directories(test_log_node_param_helper PRIVATE
        include
        ${cloudwatch_logs_common_INCLUDE_DIRS}
        ${dataflow_lite_INCLUDE_DIRS}
        ${file_management_INCLUDE_DIRS}
        ${AWSSDK_INCLUDE_DIRS}
        ${aws_common_INCLUDE_DIRS}
        ${rclcpp_INCLUDE_DIRS}
        )
target_link_libraries(test_log_node_param_helper
        ${CLOUDWATCH_LOGGER_LIBRARY_TARGET}
        )
