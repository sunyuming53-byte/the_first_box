cmake_minimum_required(VERSION 3.16)
project(realman_driver VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# RealMan SDK — discovered via find_package (Config at /opt/realman-sdk/ or cmake/)
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake")
find_package(RealManSDK REQUIRED)

# ── Library sources (GLOB_RECURSE across new subdirectory structure) ──
file(GLOB_RECURSE DRIVER_SOURCES CONFIGURE_DEPENDS
    src/core/*.cpp
    src/motion/*.cpp
    src/gripper/*.cpp
    src/state/*.cpp
    src/node/*.cpp
)

add_library(realman_driver SHARED ${DRIVER_SOURCES})
target_include_directories(realman_driver PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(realman_driver PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src   # for "core/arm_impl.hpp" private header
)

find_package(Threads REQUIRED)
target_link_libraries(realman_driver RealManSDK::RealManSDK Threads::Threads)
ament_target_dependencies(realman_driver rclcpp std_srvs tf2_ros geometry_msgs sensor_msgs)

# ── Example executables ──
add_executable(movej_test examples/hello_arm.cpp)
target_link_libraries(movej_test realman_driver)

# add_executable(external_trigger examples/external_trigger.cpp)
# target_link_libraries(external_trigger realman_driver)

add_executable(arm_node examples/arm_node.cpp)
target_link_libraries(arm_node realman_driver)
ament_target_dependencies(arm_node rclcpp std_srvs tf2_ros geometry_msgs sensor_msgs)

install(TARGETS arm_node
    DESTINATION lib/${PROJECT_NAME}
)

add_executable(gripper_test examples/gripper_test.cpp)
target_link_libraries(gripper_test realman_driver)

add_executable(joint_test examples/joint_test.cpp)
target_link_libraries(joint_test realman_driver)

add_executable(movel_test examples/movel_test.cpp)
target_link_libraries(movel_test realman_driver)

install(TARGETS realman_driver
    EXPORT realman_driver_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
install(EXPORT realman_driver_targets
    FILE realman_driverTargets.cmake
    DESTINATION share/realman_driver/cmake
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(realman_driver_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
