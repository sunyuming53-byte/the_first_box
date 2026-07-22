cmake_minimum_required(VERSION 3.22)
project(omr_vision VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/clang_tidy.cmake OPTIONAL)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui calib3d aruco objdetect)
find_package(realsense2 REQUIRED)

# GStreamer 1.0 — PRIVATE (GstStreamer is internal PIMPL, not exposed in public headers)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)

# ── Library ──
add_library(omr_vision SHARED
    src/camera/stream.cpp
    src/camera/gst_streamer.cpp
    src/camera/types.cpp
    src/capture.cpp
    src/calibration/camera_calib.cpp
    src/detection/white_label.cpp
)
target_include_directories(omr_vision PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(omr_vision PUBLIC
    ${OpenCV_LIBS}
    realsense2::realsense2
)
target_include_directories(omr_vision PRIVATE
    ${GSTREAMER_INCLUDE_DIRS}
)
target_link_libraries(omr_vision PRIVATE
    ${GSTREAMER_LIBRARIES}
)

# ── Executable: calibrate_camera ──
add_executable(calibrate_camera apps/calibrate_camera.cpp)
target_link_libraries(calibrate_camera omr_vision)

# ── Executable: detect_white_label (D435 live PnP demo) ──
add_executable(detect_white_label apps/detect_white_label.cpp)
target_link_libraries(detect_white_label omr_vision)

# ── Install ──
install(TARGETS omr_vision
    EXPORT omr_vision_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
install(TARGETS calibrate_camera detect_white_label
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(EXPORT omr_vision_targets
    FILE omr_visionTargets.cmake
    DESTINATION share/omr_vision/cmake
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_export_targets(omr_vision_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
    add_subdirectory(test)
endif()

ament_package()
