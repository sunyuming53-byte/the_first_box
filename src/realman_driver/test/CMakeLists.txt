find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(arm_node_test test_main.cpp arm_node_test.cpp
  APPEND_ENV "LD_LIBRARY_PATH=/opt/realman-sdk/lib"
)
target_link_libraries(arm_node_test realman_driver)
