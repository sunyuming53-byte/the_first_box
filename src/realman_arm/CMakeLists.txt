cmake_minimum_required(VERSION 3.16)
project(realman_arm VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(RealManSDK REQUIRED)

file(GLOB_RECURSE ARM_SOURCES CONFIGURE_DEPENDS
    src/core/*.cpp
    src/motion/*.cpp
    src/gripper/*.cpp
    src/state/*.cpp
)

add_library(realman_arm SHARED ${ARM_SOURCES})
target_include_directories(realman_arm PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${REALMAN_SDK}/include>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/sdk>
)
target_include_directories(realman_arm PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

find_package(Threads REQUIRED)
target_link_libraries(realman_arm PUBLIC
    RealManSDK::RealManSDK
    Threads::Threads
)

add_executable(movej_test examples/hello_arm.cpp)
target_link_libraries(movej_test realman_arm)

add_executable(gripper_test examples/gripper_test.cpp)
target_link_libraries(gripper_test realman_arm)

add_executable(joint_test examples/joint_test.cpp)
target_link_libraries(joint_test realman_arm)

add_executable(movel_test examples/movel_test.cpp)
target_link_libraries(movel_test realman_arm)

install(TARGETS realman_arm movej_test gripper_test joint_test movel_test
    EXPORT realman_armTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(EXPORT realman_armTargets
    FILE realman_armTargets.cmake
    NAMESPACE realman_arm::
    DESTINATION share/realman_arm/cmake
)
install(DIRECTORY include/ DESTINATION include)
install(FILES
    cmake/realman_armConfig.cmake
    cmake/RealManSDKConfig.cmake
    DESTINATION share/realman_arm/cmake
)

# ── Bundle SDK artifacts for downstream consumers ──
get_filename_component(_rm_sdk_lib_dir "${_rm_sdk_lib}" DIRECTORY)
install(DIRECTORY "${REALMAN_SDK}/include/" DESTINATION include/sdk)
install(FILES "${_rm_sdk_lib}" DESTINATION lib)
