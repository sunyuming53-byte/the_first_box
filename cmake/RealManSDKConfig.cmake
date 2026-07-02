# RealManSDKConfig.cmake
# Find module for RealMan API2 C SDK (libapi_c.so).
# Provides imported target RealManSDK::RealManSDK.
#
# Usage:
#   find_package(RealManSDK REQUIRED)
#   target_link_libraries(my_target RealManSDK::RealManSDK)
#
# SDK lookup order:
#   1. Submodule: ../third_party/RM_API2/C (alongside this Config file)
#   2. $REALMAN_SDK environment variable
#   3. /opt/realman-sdk (Docker / system install)

# 1. Submodule shipped alongside this Config file
get_filename_component(_rm_sdk_submodule "${CMAKE_CURRENT_LIST_DIR}/../third_party/RM_API2/C" ABSOLUTE)
if(EXISTS "${_rm_sdk_submodule}/include/rm_interface.h")
    set(REALMAN_SDK "${_rm_sdk_submodule}")
endif()

# 2. Environment variable override
if(NOT REALMAN_SDK)
    set(REALMAN_SDK "$ENV{REALMAN_SDK}")
endif()

# 3. System install (e.g. /opt/realman-sdk in Docker)
if(NOT REALMAN_SDK)
    set(REALMAN_SDK "/opt/realman-sdk")
endif()

set(_rm_sdk_include "${REALMAN_SDK}/include")

# libapi_c.so: flattened layout first (Docker / opt), then submodule layout
if(EXISTS "${REALMAN_SDK}/lib/libapi_c.so")
    set(_rm_sdk_lib "${REALMAN_SDK}/lib/libapi_c.so")
else()
    file(GLOB _rm_sdk_libs "${REALMAN_SDK}/linux/linux_x86_c_vv*/libapi_c.so")
    list(GET _rm_sdk_libs 0 _rm_sdk_lib)
endif()

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
