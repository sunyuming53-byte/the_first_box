cmake_minimum_required(VERSION 3.16)
project(omr_bringup VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)

# Pre-generate full robot URDF (omr.urdf.xacro → full_robot.urdf)
# The launch file Command substitution struggles with the ~24 KB xacro output;
# a static file avoids that issue and speeds up launch.
find_program(XACRO_EXECUTABLE xacro)
if(XACRO_EXECUTABLE)
  set(FULL_URDF "${CMAKE_CURRENT_BINARY_DIR}/full_robot.urdf")
  execute_process(
    COMMAND ${XACRO_EXECUTABLE}
      "${CMAKE_CURRENT_SOURCE_DIR}/urdf/omr.urdf.xacro"
      arm_ip:=192.168.1.18
      serial_port:=/dev/ttyRS485  baud_rate:=57600  slave_id:=1
      gear_ratio:=1000  screw_lead:=0.01
      m65_serial_port:=/dev/ttyBase  m65_baud_rate:=115200
      wheel_separation:=0.355  wheel_radius:=0.0625  encoder_cpr:=0
    OUTPUT_FILE ${FULL_URDF}
    RESULT_VARIABLE _xacro_rc
  )
  if(_xacro_rc EQUAL 0)
    install(FILES ${FULL_URDF} DESTINATION share/${PROJECT_NAME}/urdf)
  else()
    message(FATAL_ERROR "xacro failed for full_robot.urdf — build cannot proceed")
  endif()
else()
  message(FATAL_ERROR "xacro not found — full_robot.urdf cannot be pre-generated")
endif()
install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds)
install(DIRECTORY foxglove/ DESTINATION share/${PROJECT_NAME}/foxglove)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  ament_add_test(controller_bringup_config_test
    COMMAND "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_controller_bringup_config.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    TIMEOUT 30
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  ament_add_test(runtime_dependency_contract_test
    COMMAND "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_runtime_dependency_contract.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    TIMEOUT 30
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )

  ament_add_test(bringup_smoke_test
    COMMAND "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_bringup_smoke.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    TIMEOUT 30
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
endif()

# Suppress "CMAKE_EXPORT_COMPILE_COMMANDS was not used by the project"
set(CMAKE_EXPORT_COMPILE_COMMANDS ${CMAKE_EXPORT_COMPILE_COMMANDS})

ament_package()
