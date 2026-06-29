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

# ── Hand-eye solver unit tests (Tsai method only) ──
ament_add_gtest(hand_eye_test test_main.cpp hand_eye_test.cpp)
target_include_directories(hand_eye_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(hand_eye_test
    realman_calibration
    synthetic_poses
    ${OpenCV_LIBS}
)
target_link_options(hand_eye_test PRIVATE "LINKER:--disable-new-dtags")

# ── camera_calib_test ──
ament_add_gtest(camera_calib_test test_main.cpp camera_calib_test.cpp)
target_include_directories(camera_calib_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(camera_calib_test
    realman_calibration
    synthetic_board
    ${OpenCV_LIBS}
)
target_link_options(camera_calib_test PRIVATE "LINKER:--disable-new-dtags")

# ── tf_integration_test — HandEyeTransform load + projection ──
ament_add_gtest(tf_integration_test test_main.cpp tf_integration_test.cpp)
target_include_directories(tf_integration_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(tf_integration_test
    realman_calibration
    synthetic_poses
    ${OpenCV_LIBS}
)
target_link_options(tf_integration_test PRIVATE "LINKER:--disable-new-dtags")

# ── E2E pipeline test: Charuco images → camera calib → poses → hand-eye → YAML ──
ament_add_gtest(e2e_pipeline_test test_main.cpp e2e_pipeline_test.cpp)
target_include_directories(e2e_pipeline_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(e2e_pipeline_test
    realman_calibration
    synthetic_board
    synthetic_poses
    ${OpenCV_LIBS}
)
target_link_options(e2e_pipeline_test PRIVATE "LINKER:--disable-new-dtags")
