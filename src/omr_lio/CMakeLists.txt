cmake_minimum_required(VERSION 3.16)
project(omr_lio VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/clang_tidy.cmake OPTIONAL)

# ── Third-party ──
add_subdirectory(third_party/Sophus)

# ── Dependencies ──
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)

# Livox SDK2 is optional (only needed for AVIA LiDAR support)
find_package(livox_ros_driver2 QUIET)
if(livox_ros_driver2_FOUND)
  add_definitions(-DHAS_LIVOX)
  message(STATUS "Livox SDK2 found — AVIA LiDAR support enabled")
else()
  message(STATUS "Livox SDK2 not found — building without AVIA LiDAR support")
endif()

# GLOB all implementation .cpp files, then exclude main entry points
file(GLOB_RECURSE OMR_LIO_SRCS CONFIGURE_DEPENDS src/*.cpp)
list(FILTER OMR_LIO_SRCS EXCLUDE REGEX "src/lio_mapping\\.cpp")
list(FILTER OMR_LIO_SRCS EXCLUDE REGEX "src/estopper_main\\.cpp")

# ── Executables ──
function(add_omr_lio_exec NAME MAIN_SRC)
    add_executable(${NAME} ${MAIN_SRC} ${OMR_LIO_SRCS} third_party/ikd-Tree/ikd_Tree.cpp)
    target_include_directories(${NAME} PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        third_party
        third_party/Sophus
    )
    target_link_libraries(${NAME} Sophus)
    target_compile_definitions(${NAME} PRIVATE BOOST_BIND_GLOBAL_PLACEHOLDERS)
    ament_target_dependencies(${NAME}
        rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs
        tf2_ros tf2 pcl_conversions
    )
    target_link_libraries(${NAME}
        Eigen3::Eigen
        ${PCL_LIBRARIES}
        OpenMP::OpenMP_CXX
    )
    install(TARGETS ${NAME} DESTINATION lib/${PROJECT_NAME})
endfunction()

add_omr_lio_exec(lio_node src/lio_mapping.cpp)
add_omr_lio_exec(estopper_node src/estopper_main.cpp)

# ── Data directory (waypoint CSV output) ──
install(DIRECTORY data/ DESTINATION share/${PROJECT_NAME}/data)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# ── Python waypoint scripts ──
install(PROGRAMS scripts/record_waypoints.py DESTINATION lib/${PROJECT_NAME} RENAME record_waypoints)
install(PROGRAMS scripts/inspection_sequencer.py DESTINATION lib/${PROJECT_NAME} RENAME inspection_sequencer)
install(PROGRAMS scripts/csv_to_yaml.py DESTINATION lib/${PROJECT_NAME} RENAME csv_to_yaml)
ament_package()
