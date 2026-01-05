cmake_minimum_required(VERSION 3.20 FATAL_ERROR)

# Platform detection
if(WIN32)
    set(PLATFORM_NAME "Windows")
    set(PLATFORM_SUFFIX "win")
elseif(UNIX AND NOT APPLE)
    set(PLATFORM_NAME "Linux")
    set(PLATFORM_SUFFIX "linux")
elseif(APPLE)
    set(PLATFORM_NAME "macOS")
    set(PLATFORM_SUFFIX "macos")
endif()

# Platform-specific vcpkg toolchain file detection
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    if(WIN32)
        # Windows - vcpkg is essential
        set(VCPKG_CANDIDATES
            "C:/dev/vcpkg/vcpkg/scripts/buildsystems/vcpkg.cmake"
            "C:/vcpkg/scripts/buildsystems/vcpkg.cmake"
            "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
        )
        
        foreach(VCPKG_PATH ${VCPKG_CANDIDATES})
            if(EXISTS "${VCPKG_PATH}")
                set(CMAKE_TOOLCHAIN_FILE "${VCPKG_PATH}" CACHE STRING "")
                message(STATUS "Found vcpkg toolchain: ${VCPKG_PATH}")
                break()
            endif()
        endforeach()
        
        if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
            message(WARNING "vcpkg toolchain not found on Windows. Some dependencies may not be available.")
        endif()
    else()
        # Linux/Unix - vcpkg is optional, prefer system packages
        set(VCPKG_CANDIDATES
            "/usr/local/vcpkg/scripts/buildsystems/vcpkg.cmake"
            "$ENV{HOME}/vcpkg/scripts/buildsystems/vcpkg.cmake"
            "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
            "/opt/vcpkg/scripts/buildsystems/vcpkg.cmake"
        )
        
        foreach(VCPKG_PATH ${VCPKG_CANDIDATES})
            if(EXISTS "${VCPKG_PATH}")
                set(CMAKE_TOOLCHAIN_FILE "${VCPKG_PATH}" CACHE STRING "")
                message(STATUS "Found vcpkg toolchain: ${VCPKG_PATH}")
                break()
            endif()
        endforeach()
        
        if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
            message(STATUS "vcpkg toolchain not found. Using system packages (recommended for Linux).")
        endif()
    endif()
endif()

project(droneforge-crsf-service VERSION 1.0.0 LANGUAGES CXX)

# Enable compile commands export
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Global Windows-specific compiler definitions
if(WIN32)
    add_compile_definitions(
        NOMINMAX  # Prevent Windows.h from defining min/max macros
        WIN32_LEAN_AND_MEAN  # Reduce Windows.h bloat
        _CRT_SECURE_NO_WARNINGS  # Suppress MSVC security warnings
    )
    
    # Global Windows-specific compile options
    add_compile_options(
        /permissive-  # Strict C++ conformance
        /Zc:__cplusplus  # Enable correct __cplusplus macro value
    )
elseif(APPLE)
    # macOS-specific compiler definitions and options
    add_compile_definitions(
        _LIBCPP_ENABLE_CXX17_REMOVED_FEATURES  # Enable removed C++17 features for compatibility
    )
    
    # macOS-specific compile options
    add_compile_options(
        -Wno-deprecated-declarations  # Suppress deprecation warnings
        -stdlib=libc++  # Use libc++ standard library
        -mmacosx-version-min=10.15  # Minimum macOS version
    )
    
    # macOS-specific link options
    add_link_options(
        -stdlib=libc++
    )
endif()

# Dependencies
set(BUILD_TESTS OFF)
set(BUILD_SHARED_LIBS OFF)

# fmt configuration - simplified approach
find_package(fmt QUIET)
if(NOT fmt_FOUND)
    # Try pkgconfig on Linux
    if(UNIX AND NOT APPLE)
        find_package(PkgConfig QUIET)
        if(PkgConfig_FOUND)
            pkg_check_modules(FMT fmt)
        endif()
    endif()
    
    # If still not found, we'll create a simple target
    if(NOT FMT_FOUND)
        message(STATUS "fmt not found via package manager, will use target directly")
    endif()
endif()

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

# Source files
set(SOURCES
    src/main.cpp
    src/serial/serial.cpp
    src/crsf/crsf_manager.cpp
    src/crsf/crsf.cpp
)

# Headers
set(HEADERS
    src/common/types.h
    src/common/error.h
    src/common/static_vector.h
    src/common/thread_name.h
    src/common/scope.h
    src/common/compatibility.h
    src/serial/serial.h
    src/crsf/crsf_manager.hpp
    src/crsf/crsf.h
    src/crsf/frame.h
    src/crsf/parameter.h
    src/crsf/protocol.h
    src/crsf/crc.h
    src/client/telemetry_client.h
)

# Create executable
add_executable(droneforge-crsf-service ${SOURCES} ${HEADERS})

# Set C++ standard
set_property(TARGET droneforge-crsf-service PROPERTY CXX_STANDARD 17)
set_property(TARGET droneforge-crsf-service PROPERTY CXX_STANDARD_REQUIRED ON)

# Link libraries - conditional linking
if(fmt_FOUND)
    target_link_libraries(droneforge-crsf-service PRIVATE fmt::fmt)
    message(STATUS "Using fmt::fmt")
elseif(FMT_FOUND)
    target_include_directories(droneforge-crsf-service PRIVATE ${FMT_INCLUDE_DIRS})
    target_link_libraries(droneforge-crsf-service PRIVATE ${FMT_LIBRARIES})
    message(STATUS "Using system fmt via pkgconfig")
else()
    # Fallback - assume fmt is available somehow
    message(WARNING "fmt not found, hoping it's available at link time")
endif()

# Platform-specific libraries
if(WIN32)
    target_link_libraries(droneforge-crsf-service PRIVATE setupapi)
elseif(UNIX AND NOT APPLE)
    # Linux - needs both pthread and rt libraries
    target_link_libraries(droneforge-crsf-service PRIVATE pthread rt)
elseif(APPLE)
    # macOS - only needs pthread (rt doesn't exist on macOS)
    target_link_libraries(droneforge-crsf-service PRIVATE pthread)
endif()

# Compiler flags
if(MSVC)
    target_compile_options(droneforge-crsf-service PRIVATE 
        /W4
        /wd4996  # Disable deprecation warnings for strncpy etc.
    )
else()
    target_compile_options(droneforge-crsf-service PRIVATE 
        -Wall -Wextra -Wpedantic
        -Wno-deprecated-declarations  # Suppress deprecation warnings
    )
endif()

# Install target
install(TARGETS droneforge-crsf-service
    RUNTIME DESTINATION bin
)

# Install client header for main application
install(FILES src/client/telemetry_client.h
    DESTINATION include/droneforge-crsf
)

# Install documentation
install(FILES README.md
    DESTINATION .
) 