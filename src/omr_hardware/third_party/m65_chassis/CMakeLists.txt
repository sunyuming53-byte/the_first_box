cmake_minimum_required(VERSION 3.16)
project(m65_chassis VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost REQUIRED COMPONENTS system)

add_library(m65_chassis SHARED src/chassis.cpp)
target_include_directories(m65_chassis PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(m65_chassis PUBLIC Boost::system)

if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()
