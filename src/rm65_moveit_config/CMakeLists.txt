cmake_minimum_required(VERSION 3.16)
project(rm65_moveit_config VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)

# Suppress "CMAKE_EXPORT_COMPILE_COMMANDS was not used by the project"
set(CMAKE_EXPORT_COMPILE_COMMANDS ${CMAKE_EXPORT_COMPILE_COMMANDS})

ament_package()
