cmake_minimum_required(VERSION 3.22)
project(realman_calibration VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/clang_tidy.cmake OPTIONAL)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core calib3d highgui imgproc imgcodecs objdetect aruco)
find_package(realsense2 REQUIRED)
find_package(realman_vision REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(realman_arm REQUIRED)

# ── Libraries ──
add_library(realman_calibration SHARED
    src/collector.cpp
    src/camera_calib.cpp
    src/pose_proc.cpp
    src/hand_eye.cpp
    src/transform.cpp
)
target_include_directories(realman_calibration PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(realman_calibration PUBLIC
    realman_vision::realman_vision
    realman_arm::realman_arm
    ${OpenCV_LIBS}
)

# ── Executables ──
add_executable(collect_data apps/collect.cpp)
target_link_libraries(collect_data realman_calibration)

add_executable(calibrate_camera apps/calibrate_camera.cpp)
target_link_libraries(calibrate_camera realman_calibration)

add_executable(process_poses apps/process_poses.cpp)
target_link_libraries(process_poses realman_calibration)

add_executable(compute_hand_eye apps/compute_hand_eye.cpp)
target_link_libraries(compute_hand_eye realman_calibration)

add_executable(run_pipeline apps/run_pipeline.cpp)
target_link_libraries(run_pipeline realman_calibration)

add_executable(calib_node apps/calib_node.cpp)
target_link_libraries(calib_node realman_calibration)
ament_target_dependencies(calib_node rclcpp sensor_msgs std_msgs std_srvs tf2_ros geometry_msgs visualization_msgs)

# ── Install ──
install(TARGETS realman_calibration
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
install(TARGETS collect_data calibrate_camera process_poses compute_hand_eye run_pipeline calib_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_package()
