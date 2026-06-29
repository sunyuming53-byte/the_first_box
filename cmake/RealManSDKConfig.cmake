# RealManSDKConfig.cmake
# Find module for RealMan API2 C SDK (libapi_c.so).
# Provides imported target RealManSDK::RealManSDK.
#
# Usage:
#   find_package(RealManSDK REQUIRED)
#   target_link_libraries(my_target RealManSDK::RealManSDK)
#
# The SDK is expected at ${REALMAN_SDK} or /opt/realman-sdk,
# with headers under include/ and libapi_c.so under lib/.

if(NOT DEFINED REALMAN_SDK)
    set(REALMAN_SDK "$ENV{REALMAN_SDK}")
endif()
if(NOT REALMAN_SDK)
    set(REALMAN_SDK "/opt/realman-sdk")
endif()

set(_rm_sdk_include "${REALMAN_SDK}/include")
set(_rm_sdk_lib     "${REALMAN_SDK}/lib/libapi_c.so")

if(NOT EXISTS "${_rm_sdk_include}/rm_interface.h")
    message(FATAL_ERROR "RealManSDK headers not found at ${_rm_sdk_include}")
endif()
if(NOT EXISTS "${_rm_sdk_lib}")
    message(FATAL_ERROR "RealManSDK library not found at ${_rm_sdk_lib}")
endif()

if(NOT TARGET RealManSDK::RealManSDK)
    add_library(RealManSDK::RealManSDK SHARED IMPORTED)
    set_target_properties(RealManSDK::RealManSDK PROPERTIES
        IMPORTED_LOCATION "${_rm_sdk_lib}"
        INTERFACE_INCLUDE_DIRECTORIES "${_rm_sdk_include}"
    )
endif()

set(RealManSDK_FOUND TRUE)
