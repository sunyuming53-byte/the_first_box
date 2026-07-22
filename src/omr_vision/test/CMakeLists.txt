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

# ── synthetic_poses — test-only library ──
add_library(synthetic_poses STATIC synthetic_poses.cpp)
target_include_directories(synthetic_poses PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(synthetic_poses PUBLIC ${OpenCV_LIBS})

# ── camera_calib_test ──
ament_add_gtest(camera_calib_test test_main.cpp camera_calib_test.cpp)
target_include_directories(camera_calib_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(camera_calib_test
    omr_vision
    synthetic_board
    ${OpenCV_LIBS}
)

# ── white_label_test ──
ament_add_gtest(white_label_test test_main.cpp white_label_test.cpp)
target_include_directories(white_label_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(white_label_test
    omr_vision
    ${OpenCV_LIBS}
)
