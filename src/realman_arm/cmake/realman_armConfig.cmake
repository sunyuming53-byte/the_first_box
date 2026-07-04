# Create RealManSDK imported target from the bundled SDK
set(_rm_config_dir "${CMAKE_CURRENT_LIST_DIR}")
set(_rm_sdk_include "${_rm_config_dir}/../../../include/sdk")
set(_rm_sdk_lib "${_rm_config_dir}/../../../lib/libapi_c.so")

if(NOT TARGET RealManSDK::RealManSDK)
    add_library(RealManSDK::RealManSDK SHARED IMPORTED)
    set_target_properties(RealManSDK::RealManSDK PROPERTIES
        IMPORTED_LOCATION "${_rm_sdk_lib}"
        INTERFACE_INCLUDE_DIRECTORIES "${_rm_sdk_include}"
    )
endif()

include("${CMAKE_CURRENT_LIST_DIR}/realman_armTargets.cmake")
