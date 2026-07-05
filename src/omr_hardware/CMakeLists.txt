cmake_minimum_required(VERSION 3.16)
project(omr_hardware VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/clang_tidy.cmake OPTIONAL)

# ── Submodules (in-tree, no find_package needed) ──
set(REALMAN_ARM_AS_SUBMODULE ON CACHE BOOL "" FORCE)
add_subdirectory(third_party/realman_arm EXCLUDE_FROM_ALL)
add_subdirectory(third_party/dais_motor EXCLUDE_FROM_ALL)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
    src/arm_system.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
    hardware_interface pluginlib rclcpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC realman_arm dais_motor)

pluginlib_export_plugin_description_file(hardware_interface plugins.xml)

install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
# realman_arm is a submodule dependency needed by downstream calibration
install(TARGETS realman_arm dais_motor
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY third_party/realman_arm/include/ DESTINATION include/realman_arm)

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_package()
