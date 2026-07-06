cmake_minimum_required(VERSION 3.16)
project(omr_controller VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/clang_tidy.cmake OPTIONAL)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(omr_vision REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(OpenCV REQUIRED)

file(GLOB_RECURSE LIB_SOURCES src/*.cpp)
list(REMOVE_ITEM LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)

add_library(${PROJECT_NAME} SHARED ${LIB_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
    ament_index_cpp
    behaviortree_cpp
    control_msgs
    geometry_msgs
    omr_vision
    rclcpp
    rclcpp_action
    sensor_msgs
    std_msgs
    trajectory_msgs
)
target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS})

install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY bt_xml/ DESTINATION share/${PROJECT_NAME}/bt_xml)

add_executable(task_orchestrator src/main.cpp)
target_link_libraries(task_orchestrator ${PROJECT_NAME})
install(TARGETS task_orchestrator DESTINATION lib/${PROJECT_NAME})

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(realsense2 REQUIRED)
    ament_add_gtest(types_test test/test_main.cpp test/types_test.cpp)
    target_link_libraries(types_test ${PROJECT_NAME} ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(gripper_client_test test/test_main.cpp test/gripper_client_test.cpp)
    target_link_libraries(gripper_client_test ${PROJECT_NAME} ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(motor_client_test test/test_main.cpp test/motor_client_test.cpp)
    target_link_libraries(motor_client_test ${PROJECT_NAME} ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(base_client_test test/test_main.cpp test/base_client_test.cpp)
    target_link_libraries(base_client_test ${PROJECT_NAME} ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(arm_client_test test/test_main.cpp test/arm_client_test.cpp)
    target_link_libraries(arm_client_test ${PROJECT_NAME} ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(vision_client_test test/test_main.cpp test/vision_client_test.cpp)
    target_include_directories(vision_client_test PRIVATE test)
    target_link_libraries(vision_client_test ${PROJECT_NAME} ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(bt_factory_test test/test_main.cpp test/bt_factory_test.cpp)
    target_include_directories(bt_factory_test PRIVATE test)
    target_link_libraries(bt_factory_test ${PROJECT_NAME} ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(bt_xml_test test/test_main.cpp test/bt_xml_test.cpp)
    target_include_directories(bt_xml_test PRIVATE test)
    target_link_libraries(bt_xml_test ${PROJECT_NAME} ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(client_integration_test test/test_main.cpp test/client_integration_test.cpp)
    target_include_directories(client_integration_test PRIVATE test)
    target_link_libraries(client_integration_test ${PROJECT_NAME} ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(orchestrator_test test/test_main.cpp test/orchestrator_test.cpp)
    target_include_directories(orchestrator_test PRIVATE test)
    target_link_libraries(orchestrator_test ${PROJECT_NAME} ${OpenCV_LIBS} realsense2::realsense2)

    ament_add_gtest(orchestrator_integration_test test/test_main.cpp test/orchestrator_integration_test.cpp)
    target_include_directories(orchestrator_integration_test PRIVATE test)
    target_link_libraries(orchestrator_integration_test ${PROJECT_NAME} ${OpenCV_LIBS} realsense2::realsense2)
endif()

ament_package()
