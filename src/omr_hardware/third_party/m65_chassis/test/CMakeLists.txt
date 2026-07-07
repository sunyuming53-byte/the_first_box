find_package(GTest REQUIRED)

add_executable(test_serial_framing test_serial_framing.cpp)
target_include_directories(test_serial_framing PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../src)
target_link_libraries(test_serial_framing m65_chassis GTest::GTest)
add_test(NAME serial_framing COMMAND test_serial_framing)
set_tests_properties(serial_framing PROPERTIES
    ENVIRONMENT "LD_LIBRARY_PATH=$<TARGET_FILE_DIR:m65_chassis>:$ENV{LD_LIBRARY_PATH}")
