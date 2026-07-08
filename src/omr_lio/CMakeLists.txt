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

file(GLOB_RECURSE OMR_LIO_SRCS src/*.cpp CONFIGURE_DEPENDS)

# ── Executables ──
function(add_omr_lio_exec NAME)
    add_executable(${NAME} ${OMR_LIO_SRCS} third_party/ikd-Tree/ikd_Tree.cpp)
    target_include_directories(${NAME} PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        third_party/ikd-Tree
    )
    target_link_libraries(${NAME} PRIVATE Sophus)
    ament_target_dependencies(${NAME} PRIVATE
        rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs
        tf2_ros tf2 pcl_conversions
    )
    target_link_libraries(${NAME} PRIVATE
        Eigen3::Eigen
        ${PCL_LIBRARIES}
        OpenMP::OpenMP_CXX
    )
    install(TARGETS ${NAME} DESTINATION lib/${PROJECT_NAME})
endfunction()

add_omr_lio_exec(lio_node)
add_omr_lio_exec(estopper_node)

# ── Data directory (waypoint CSV output) ──
install(DIRECTORY data/ DESTINATION share/${PROJECT_NAME}/data)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
