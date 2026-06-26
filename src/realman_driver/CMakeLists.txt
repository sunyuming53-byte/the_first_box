cmake_minimum_required(VERSION 3.16)
project(realman_driver VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# RealMan SDK path — defaults to workspace third_party/RM_API2/C
if(NOT DEFINED REALMAN_SDK)
    set(REALMAN_SDK "$ENV{REALMAN_SDK}")
endif()
if(NOT REALMAN_SDK)
    set(REALMAN_SDK "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/RM_API2/C")
endif()
if(NOT EXISTS "${REALMAN_SDK}")
    message(FATAL_ERROR "REALMAN_SDK not found at ${REALMAN_SDK}. Pass -DREALMAN_SDK=/path/to/RM_API2/C")
endif()

# SDK library path — override with -DREALMAN_SDK_LIB if needed
if(NOT DEFINED REALMAN_SDK_LIB)
    file(GLOB RM_SDK_LIB_CANDIDATES "${REALMAN_SDK}/linux/linux_x86_c_v*/libapi_c.so")
    list(GET RM_SDK_LIB_CANDIDATES 0 REALMAN_SDK_LIB)
endif()
if(NOT REALMAN_SDK_LIB)
    message(FATAL_ERROR "Could not find libapi_c.so under ${REALMAN_SDK}/linux/")
endif()

add_library(rm_api SHARED IMPORTED)
set_target_properties(rm_api PROPERTIES
    IMPORTED_LOCATION "${REALMAN_SDK_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${REALMAN_SDK}/include"
)

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
target_link_libraries(realman_driver rm_api Threads::Threads)
ament_target_dependencies(realman_driver rclcpp)

# add_executable(hello_arm examples/hello_arm.cpp)
# target_link_libraries(hello_arm realman_driver)

# add_executable(external_trigger examples/external_trigger.cpp)
# target_link_libraries(external_trigger realman_driver)

# add_executable(arm_node examples/arm_node.cpp)
# target_link_libraries(arm_node realman_driver)

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
