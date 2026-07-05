cmake_minimum_required(VERSION 3.22)
project(omr_vision VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/clang_tidy.cmake OPTIONAL)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui)
find_package(realsense2 REQUIRED)

# ── Library ──
add_library(omr_vision SHARED
    src/camera/stream.cpp
    src/camera/types.cpp
    src/capture.cpp
)
target_include_directories(omr_vision PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(omr_vision PUBLIC
    ${OpenCV_LIBS}
    realsense2::realsense2
)

# ── Install ──
install(TARGETS omr_vision
    EXPORT omr_vision_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
install(EXPORT omr_vision_targets
    FILE omr_visionTargets.cmake
    DESTINATION share/omr_vision/cmake
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(omr_vision_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
