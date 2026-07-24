cmake_minimum_required(VERSION 3.16)
project(omr_controller VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/clang_tidy.cmake OPTIONAL)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(omr_vision REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

file(GLOB_RECURSE LIB_SOURCES src/*.cpp)
list(APPEND LIB_SOURCES apps/remote_control_node.cpp)
list(REMOVE_ITEM LIB_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveArm.srv"
  "srv/MoveRail.srv"
)

add_library(${PROJECT_NAME}_lib SHARED ${LIB_SOURCES})
set_target_properties(${PROJECT_NAME}_lib PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)

# realman_arm linkage from omr_hardware (needed for calibration collector)
find_package(omr_hardware REQUIRED)
get_filename_component(_omr_hw_prefix "${omr_hardware_DIR}/../../.." ABSOLUTE)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
    $<BUILD_INTERFACE:${_omr_hw_prefix}/include>
    $<BUILD_INTERFACE:${_omr_hw_prefix}/include/realman_arm>
)
find_library(_realman_arm_lib realman_arm
    PATHS "${_omr_hw_prefix}/lib" NO_DEFAULT_PATH
)
find_library(_api_c_lib api_c
    PATHS /opt/realman-sdk/lib
)
target_link_libraries(${PROJECT_NAME}_lib PUBLIC ${_realman_arm_lib})
if(_api_c_lib)
  target_link_libraries(${PROJECT_NAME}_lib PUBLIC ${_api_c_lib})
endif()

# rail_control from omr_hardware (GuidewayClient backend; PIMPL header, so no
# photogate/dais_motor headers are needed here)
find_library(_rail_control_lib rail_control
    PATHS "${_omr_hw_prefix}/lib" NO_DEFAULT_PATH
)
target_link_libraries(${PROJECT_NAME}_lib PUBLIC ${_rail_control_lib})

ament_target_dependencies(${PROJECT_NAME}_lib PUBLIC
    ament_index_cpp
    behaviortree_cpp
    control_msgs
    cv_bridge
    diagnostic_msgs
    diagnostic_updater
    geometry_msgs
    moveit_msgs
    moveit_ros_planning_interface
    nav_msgs
    omr_vision
    rcl_interfaces
    rosidl_default_runtime
    rclcpp
    rclcpp_action
    sensor_msgs
    std_msgs
    tf2
    trajectory_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_lib PUBLIC "${cpp_typesupport_target}")

target_link_libraries(${PROJECT_NAME}_lib PUBLIC ${OpenCV_LIBS})

install(TARGETS ${PROJECT_NAME}_lib
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY srv/ DESTINATION share/${PROJECT_NAME}/srv)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY bt_xml/ DESTINATION share/${PROJECT_NAME}/bt_xml)

add_executable(task_orchestrator src/main.cpp)
target_link_libraries(task_orchestrator ${PROJECT_NAME}_lib)
install(TARGETS task_orchestrator DESTINATION lib/${PROJECT_NAME})

add_executable(robot_diagnostics apps/robot_diagnostics_main.cpp)
target_link_libraries(robot_diagnostics ${PROJECT_NAME}_lib)
install(TARGETS robot_diagnostics DESTINATION lib/${PROJECT_NAME})

add_executable(remote_control apps/remote_control.cpp)
target_link_libraries(remote_control ${PROJECT_NAME}_lib)
install(TARGETS remote_control DESTINATION lib/${PROJECT_NAME})

# ── Calibration executables ──
add_executable(calib_node apps/calib_node.cpp)
target_link_libraries(calib_node ${PROJECT_NAME}_lib ${_realman_arm_lib})
ament_target_dependencies(calib_node rclcpp rcl_interfaces sensor_msgs std_msgs std_srvs tf2_ros
                          geometry_msgs visualization_msgs)

add_executable(collect_data apps/collect.cpp)
target_link_libraries(collect_data ${PROJECT_NAME}_lib)

add_executable(compute_hand_eye apps/compute_hand_eye.cpp)
target_link_libraries(compute_hand_eye ${PROJECT_NAME}_lib)

add_executable(process_poses apps/process_poses.cpp)
target_link_libraries(process_poses ${PROJECT_NAME}_lib)

add_executable(run_pipeline apps/run_pipeline.cpp)
target_link_libraries(run_pipeline ${PROJECT_NAME}_lib)

install(TARGETS calib_node collect_data compute_hand_eye process_poses run_pipeline
    RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_libraries(${PROJECT_NAME}_lib)
add_executable(dais_positioning_node apps/dais_positioning_node.cpp)
target_link_libraries(dais_positioning_node ${PROJECT_NAME}_lib)
install(TARGETS dais_positioning_node RUNTIME DESTINATION lib/${PROJECT_NAME})
ament_export_include_directories(include)

# Manual hardware smoke for GuidewayClientImpl (NOT part of colcon test / CI).
# Source lives under test/ but is an installable CLI, not a gtest.
add_executable(guideway_client_hw_test test/guideway_client_hw_test.cpp)
target_link_libraries(guideway_client_hw_test ${PROJECT_NAME}_lib)
install(TARGETS guideway_client_hw_test RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    # Process the RM65 URDF xacro for collision_avoidance_test
    set(XACRO_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../rm65_moveit_config/urdf/rm65_moveit.urdf.xacro")
    if(EXISTS "${XACRO_SRC}")
      execute_process(
        COMMAND xacro "${XACRO_SRC}"
        OUTPUT_VARIABLE URDF_CONTENT
        RESULT_VARIABLE XACRO_RESULT
        OUTPUT_STRIP_TRAILING_WHITESPACE
      )
      if(XACRO_RESULT EQUAL 0)
        file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/rm65_urdf.h"
          "// Auto-generated by CMake from rm65_moveit.urdf.xacro\n"
          "#pragma once\n"
          "namespace omr_controller::test {\n"
          "inline const char* kRm65Urdf = R\"urdf(\n${URDF_CONTENT}\n)urdf\";\n"
          "}\n")
      else()
        message(WARNING "xacro failed on ${XACRO_SRC} — collision_avoidance_test will skip URDF-dependent checks")
      endif()
    else()
      message(WARNING "${XACRO_SRC} not found — collision_avoidance_test will skip URDF-dependent checks")
    endif()

    # Embed SRDF at compile time so collision_avoidance_test does not need
    # rm65_moveit_config as a runtime dependency (avoids a package.xml cycle:
    # rm65_moveit_config → omr_bringup → omr_controller → rm65_moveit_config).
    set(SRDF_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../rm65_moveit_config/config/rm65.srdf")
    if(EXISTS "${SRDF_SRC}")
      file(READ "${SRDF_SRC}" SRDF_CONTENT)
      file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/rm65_srdf.h"
        "// Auto-generated by CMake from rm65_moveit_config/config/rm65.srdf\n"
        "#pragma once\n"
        "namespace omr_controller::test {\n"
        "inline const char* kRm65Srdf = R\"srdf(\n${SRDF_CONTENT}\n)srdf\";\n"
        "}\n")
    else()
      message(WARNING "${SRDF_SRC} not found — collision_avoidance_test will skip SRDF-dependent checks")
    endif()

    ament_add_gtest(types_test test/test_main.cpp test/types_test.cpp)
    target_link_libraries(types_test ${PROJECT_NAME}_lib ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(door_math_test test/test_main.cpp test/door_math_test.cpp)
    target_include_directories(door_math_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(door_math_test ${OpenCV_LIBS})

    ament_add_gtest(geometry_utils_test test/test_main.cpp test/geometry_utils_test.cpp)
    target_include_directories(geometry_utils_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(geometry_utils_test ${PROJECT_NAME}_lib ${OpenCV_LIBS})

    ament_add_gtest(gripper_client_test test/test_main_ros.cpp test/gripper_client_test.cpp)
    target_link_libraries(gripper_client_test ${PROJECT_NAME}_lib ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(motor_client_test test/test_main.cpp test/motor_client_test.cpp)
    target_link_libraries(motor_client_test ${PROJECT_NAME}_lib ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(guideway_client_test test/test_main.cpp test/guideway_client_test.cpp)
    target_link_libraries(guideway_client_test ${PROJECT_NAME}_lib ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(motor_client_impl_test test/test_main_ros.cpp test/motor_client_impl_test.cpp)
    target_link_libraries(motor_client_impl_test ${PROJECT_NAME}_lib ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(base_client_test test/test_main_ros.cpp test/base_client_test.cpp)
    target_link_libraries(base_client_test ${PROJECT_NAME}_lib ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(arm_client_test test/test_main_ros.cpp test/arm_client_test.cpp)
    target_link_libraries(arm_client_test ${PROJECT_NAME}_lib ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(remote_control_test test/test_main_ros.cpp test/remote_control_test.cpp)
    target_include_directories(remote_control_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(remote_control_test ${PROJECT_NAME}_lib ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(vision_client_test test/test_main.cpp test/vision_client_test.cpp)
    target_include_directories(vision_client_test PRIVATE test)
    target_link_libraries(vision_client_test ${PROJECT_NAME}_lib ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(bt_factory_test test/test_main_ros.cpp test/bt_factory_test.cpp)
    target_include_directories(bt_factory_test PRIVATE test)
    target_link_libraries(bt_factory_test ${PROJECT_NAME}_lib ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(bt_xml_test test/test_main.cpp test/bt_xml_test.cpp)
    target_include_directories(bt_xml_test PRIVATE test)
    target_link_libraries(bt_xml_test ${PROJECT_NAME}_lib ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(client_integration_test test/test_main_ros.cpp test/client_integration_test.cpp)
    target_include_directories(client_integration_test PRIVATE test)
    target_link_libraries(client_integration_test ${PROJECT_NAME}_lib ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(orchestrator_test test/test_main_ros.cpp test/orchestrator_test.cpp)
    target_include_directories(orchestrator_test PRIVATE test)
    target_link_libraries(orchestrator_test ${PROJECT_NAME}_lib ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(orchestrator_integration_test test/test_main_ros.cpp test/orchestrator_integration_test.cpp)
    target_include_directories(orchestrator_integration_test PRIVATE test)
    target_link_libraries(orchestrator_integration_test ${PROJECT_NAME}_lib ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(door_trajectory_node_test test/test_main_ros.cpp test/door_trajectory_node_test.cpp)
    target_link_libraries(door_trajectory_node_test ${PROJECT_NAME}_lib ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(moveit_integration_test test/test_main_ros.cpp test/moveit_integration_test.cpp)
    target_link_libraries(moveit_integration_test ${PROJECT_NAME}_lib ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(trajectory_sequence_test test/test_main_ros.cpp test/trajectory_sequence_test.cpp)
    target_link_libraries(trajectory_sequence_test ${PROJECT_NAME}_lib ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(door_trajectory_edge_test test/test_main_ros.cpp test/door_trajectory_edge_test.cpp)
    target_link_libraries(door_trajectory_edge_test ${PROJECT_NAME}_lib ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(door_collision_test test/test_main_ros.cpp test/door_collision_test.cpp)
    target_link_libraries(door_collision_test ${PROJECT_NAME}_lib ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(collision_avoidance_test test/test_main_ros.cpp test/collision_avoidance_test.cpp)
    target_include_directories(collision_avoidance_test PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
    target_link_libraries(collision_avoidance_test ${PROJECT_NAME}_lib ${OpenCV_LIBS})

    ament_add_gtest(topic_camera_adapter_test test/test_main_ros.cpp test/topic_camera_adapter_test.cpp)
    target_include_directories(topic_camera_adapter_test PRIVATE test)
    target_link_libraries(topic_camera_adapter_test ${PROJECT_NAME}_lib ${OpenCV_LIBS})

    # ── Calibration tests ──
    # synthetic_poses (minimal copy for hand_eye/tf_integration tests)
    add_library(calib_synthetic_poses STATIC test/calib/synthetic_poses.cpp)
    target_include_directories(calib_synthetic_poses PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/test/calib)
    target_link_libraries(calib_synthetic_poses PUBLIC ${OpenCV_LIBS})

    ament_add_gtest(pose_proc_test test/test_main.cpp test/calib/pose_proc_test.cpp)
    target_include_directories(pose_proc_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(pose_proc_test ${PROJECT_NAME}_lib ${OpenCV_LIBS})

    ament_add_gtest(hand_eye_test test/test_main.cpp test/calib/hand_eye_test.cpp)
    target_include_directories(hand_eye_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(hand_eye_test ${PROJECT_NAME}_lib calib_synthetic_poses ${OpenCV_LIBS})

    ament_add_gtest(tf_integration_test test/test_main.cpp test/calib/tf_integration_test.cpp)
    target_include_directories(tf_integration_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(tf_integration_test ${PROJECT_NAME}_lib calib_synthetic_poses ${OpenCV_LIBS})

    # e2e_pipeline_test: SKIPPED — requires synthetic_board (233-line image
    # renderer in omr_vision/test/) which is too large to copy. Re-enable when
    # a shared test library solution is implemented across packages.
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
