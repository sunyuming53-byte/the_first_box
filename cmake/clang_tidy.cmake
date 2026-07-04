# RealMan ROS2 Workspace — Opt-in clang-tidy CMake integration
#
# Enable with:
#   colcon build --cmake-args -DCLANG_TIDY=ON
#
# CMake automatically passes target compile flags (include dirs, defines,
# -std=c++23) to clang-tidy. The .clang-tidy file is discovered from the
# workspace root via --config-file.

option(CLANG_TIDY "Enable clang-tidy static analysis during build" OFF)

if(CLANG_TIDY)
  find_program(CLANG_TIDY_EXE NAMES clang-tidy clang-tidy-19 clang-tidy-18 clang-tidy-17 clang-tidy-16 clang-tidy-15 clang-tidy-14)

  if(CLANG_TIDY_EXE)
    message(STATUS "clang-tidy enabled: ${CLANG_TIDY_EXE}")

    set(CMAKE_CXX_CLANG_TIDY
      "${CLANG_TIDY_EXE}"
      "--config-file=${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-tidy"
    )
  else()
    message(WARNING "CLANG_TIDY=ON requested but clang-tidy not found")
  endif()
endif()
