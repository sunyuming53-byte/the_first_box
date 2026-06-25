cmake_minimum_required(VERSION 3.16)
project(realman-pipeline VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# RealMan SDK path
if(NOT DEFINED REALMAN_SDK)
    set(REALMAN_SDK "$ENV{REALMAN_SDK}")
endif()
if(NOT REALMAN_SDK)
    message(FATAL_ERROR "REALMAN_SDK not set. Pass -DREALMAN_SDK=/path/to/RM_API2/C")
endif()

add_library(rm_api SHARED IMPORTED)
set_target_properties(rm_api PROPERTIES
    IMPORTED_LOCATION "${REALMAN_SDK}/linux/linux_x86_c_vv1.1.5/libapi_c.so"
    INTERFACE_INCLUDE_DIRECTORIES "${REALMAN_SDK}/include"
)

add_library(realman-pipeline SHARED
    src/arm.cpp
    src/arm_node.cpp
    src/error.cpp
)
target_include_directories(realman-pipeline PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(realman-pipeline PUBLIC
    rclcpp::rclcpp
    rm_api
    pthread
)
ament_target_dependencies(realman-pipeline rclcpp)

add_executable(hello_arm examples/hello_arm.cpp)
target_link_libraries(hello_arm realman-pipeline)

add_executable(external_trigger examples/external_trigger.cpp)
target_link_libraries(external_trigger realman-pipeline)

add_executable(arm_node examples/arm_node.cpp)
target_link_libraries(arm_node realman-pipeline)

install(TARGETS realman-pipeline
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

ament_package()
