find_package(ament_cmake_gtest REQUIRED)
find_package(OpenCV REQUIRED)

# ── Synthetic board image generator (shared lib for tests) ──
add_library(synthetic_board SHARED
  synthetic_board.cpp
)
target_include_directories(synthetic_board PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(synthetic_board PUBLIC
  ${OpenCV_LIBS}
)
target_compile_features(synthetic_board PUBLIC cxx_std_23)

# ── Smoke-test: verify synthetic images pass chessboard detection ──
add_executable(verify_synthetic verify_synthetic.cpp)
target_link_libraries(verify_synthetic synthetic_board ${OpenCV_LIBS})

ament_add_gtest(pose_proc_test test_main.cpp pose_proc_test.cpp)
target_include_directories(pose_proc_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(pose_proc_test
    realman_calibration
    ${OpenCV_LIBS}
)

# Use RPATH (not RUNPATH) so transitive dependencies (librealsense2) are found.
target_link_options(pose_proc_test PRIVATE "LINKER:--disable-new-dtags")

# ── synthetic_poses — test-only library ──
add_library(synthetic_poses STATIC synthetic_poses.cpp)
target_include_directories(synthetic_poses PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(synthetic_poses PUBLIC ${OpenCV_LIBS})
