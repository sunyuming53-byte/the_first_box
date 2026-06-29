cmake_minimum_required(VERSION 3.16)
project(realman_driver VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# RealMan SDK — discovered via find_package (Config at /opt/realman-sdk/ or cmake/)
list(CMAKE_PREFIX_PATH APPEND "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake")
find_package(RealManSDK REQUIRED)

add_library(realman_driver SHARED
    src/arm.cpp
    src/error.cpp
    # src/arm_node.cpp  # TODO: Task 6
)
target_include_directories(realman_driver PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
find_package(Threads REQUIRED)
target_link_libraries(realman_driver RealManSDK::RealManSDK Threads::Threads)
ament_target_dependencies(realman_driver rclcpp)

# add_executable(hello_arm examples/hello_arm.cpp)
# target_link_libraries(hello_arm realman_driver)

# add_executable(external_trigger examples/external_trigger.cpp)
# target_link_libraries(external_trigger realman_driver)

add_executable(arm_node examples/arm_node.cpp)
target_link_libraries(arm_node realman_driver)

add_executable(gripper_test examples/gripper_test.cpp)
target_link_libraries(gripper_test realman_driver)

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

ament_package()
