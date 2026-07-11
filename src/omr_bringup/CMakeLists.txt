cmake_minimum_required(VERSION 3.16)
project(omr_bringup VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds)

# Suppress "CMAKE_EXPORT_COMPILE_COMMANDS was not used by the project"
set(CMAKE_EXPORT_COMPILE_COMMANDS ${CMAKE_EXPORT_COMPILE_COMMANDS})

ament_package()
