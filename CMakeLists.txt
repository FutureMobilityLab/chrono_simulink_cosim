cmake_minimum_required(VERSION 3.15)
cmake_policy(SET CMP0091 NEW)
project(chrono_simulink_cosim CXX)

# Find External Dependencies
# Use find_package to get Chrono and Pybind11 targets
find_package(Chrono CONFIG REQUIRED COMPONENTS Vehicle Irrlicht FMI)
if (NOT Chrono_FOUND)
    message(FATAL_ERROR "Project Chrono not found. Set Chrono_DIR.")
endif()

# Change chrono data directories to the one in this project.
set(CHRONO_DATA_DIR "${PROJECT_SOURCE_DIR}/data/")
set(CHRONO_VEHICLE_DATA_DIR "${PROJECT_SOURCE_DIR}/data/vehicle/")
include_directories(${CHRONO_INCLUDE_DIRS})

# When using static libraries, we need additional definitions
if(NOT BUILD_SHARED_LIBS)
  add_definitions(-DCHRONO_STATIC_LINK)
endif()

# Depending on the platform, static linking may require additional libraries
if(UNIX AND NOT APPLE)
  find_package(Threads REQUIRED)
  list(APPEND CHRONO_LIBRARIES ${CMAKE_THREAD_LIBS_INIT} dl)
  if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    list(APPEND CHRONO_LIBRARIES rt)
  endif()
endif()

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

#--------------------------------------------------------------
# Enable creation of "application bundles" on MacOSX.
#--------------------------------------------------------------

# This is necessary for any Irrlicht-based project (like the example here).
# For OpenGL-based or non-graphics projects, this is optional and the block
# below can be removed (or else explcitly set CMAKE_MACOSX_BUNDLE to 'OFF').
#
# If creating application bundles, the build output will be named 'myexe.app'.
# Use the convenience script 'run_app.sh' available under 'contrib/appbundle-macosx/'
# to run:
#     start_demo.sh myexe.app

if(APPLE)
    set(CMAKE_MACOSX_BUNDLE ON)
endif()

#--------------------------------------------------------------
# Add path to Chrono headers and to headers of all dependencies
# of the requested modules.
#--------------------------------------------------------------

#--------------------------------------------------------------
# Tweaks to disable some warnings with MSVC
#--------------------------------------------------------------
if(MSVC)
    # Define NOMINMAX to prevent Windows.h from defining min/max macros
    # Define _CRT_SECURE_NO_DEPRECATE and _SCL_SECURE_NO_DEPRECATE for C++ standard library warnings
    add_compile_definitions(NOMINMAX _CRT_SECURE_NO_DEPRECATE _SCL_SECURE_NO_DEPRECATE)

    # Suppress specific warnings that often arise from third-party libraries or common C++ patterns
    # /wd4250 - "inherits ... via dominance" (virtual inheritance issues)
    # /wd4251 - "class 'A' needs to have dll-interface"
    # /wd4275 - "non dll-interface class 'A' used as base for dll-interface class 'B'"
    add_compile_options(/wd4250 /wd4251 /wd4275)
endif()

#--------------------------------------------------------------
# Add path to Chrono headers and to headers of all dependencies
# of the requested modules.
#--------------------------------------------------------------

include_directories(${CHRONO_INCLUDE_DIRS})

#-----------------------------------------------------------------------------
# Fix for VS 2017 15.8 and newer to handle alignment specification with Eigen
#-----------------------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  if(MSVC AND ${MSVC_VERSION} GREATER_EQUAL 1915)
    add_definitions( "-D_ENABLE_EXTENDED_ALIGNED_STORAGE" )
  endif()
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON) # Essential for shared libraries

# Configure output directories for a clean build
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Copy Chrono DLLS (add_DLL_copy_command is a CMake helper function from Chrono).
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
add_DLL_copy_command("${CHRONO_DLL_NAMES}" "${CHRONO_DEPENDENCY_DLLS}")

#--------------------------------------------------------------
# Define helper functions to simplify CMakeLists.txt throughout
# this project.
#--------------------------------------------------------------

# Define a variable for the project root include directory.
# This points to C:/.../chrono_simulink_cosim/ (your top-level project directory).
# It allows C++ files to use #include "src/module_name/header.h" everywhere.
set(PROJECT_ROOT_INCLUDE_DIR $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>)

# --- Helper Function: add_project_root_includes ---
# This function adds the project root to a target's include paths.
# It's now called internally by create_library and create_executable.
function(add_project_root_includes_internal TARGET_NAME)
    target_include_directories(${TARGET_NAME} PRIVATE ${PROJECT_ROOT_INCLUDE_DIR})
endfunction()

# --- Helper Function: create_library ---
# This function simplifies the creation of libraries in the project.
# Usage:
# create_library(<name> <type>
#              SRCS <source_files...>
#              HDRS <header_files...>
#              DEPS <dependency_targets...>
#              [USES_CHRONO]
# )
function(create_library LIB_NAME LIB_TYPE)
    cmake_parse_arguments(
        "LIB" # prefix for parsed arguments
        "USES_CHRONO" # boolean arguments (flags)
        "" # single-value arguments
        "SRCS;HDRS;DEPS" # multi-value arguments (lists)
        ${ARGN} # arguments to parse
    )

    # Add the library itself
    add_library(${LIB_NAME} ${LIB_TYPE} ${LIB_SRCS})

    # Apply project root includes for the "src/..." include style
    add_project_root_includes_internal(${LIB_NAME})

    # Add public headers for this library
    if(LIB_HDRS)
        target_include_directories(${LIB_NAME} PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}> # Current module's directory
        )
    endif()

    # Link dependencies
    if(LIB_DEPS)
        target_link_libraries(${LIB_NAME} PRIVATE ${LIB_DEPS})
    endif()

    # Link Chrono libraries if requested
    if(LIB_USES_CHRONO)
        target_link_libraries(${LIB_NAME} PRIVATE ${CHRONO_LIBRARIES})
        if(CHRONO_DATA_DIR)
            target_compile_definitions(${LIB_NAME} PRIVATE CHRONO_DATA_DIR="${CHRONO_DATA_DIR}")
        endif()
        if(CHRONO_VEHICLE_DATA_DIR)
            target_compile_definitions(${LIB_NAME} PRIVATE CHRONO_VEHICLE_DATA_DIR="${CHRONO_VEHICLE_DATA_DIR}")
        endif()
    endif()

endfunction()

# --- Helper Function: create_executable ---
# This function simplifies the creation of executables in the project.
# Usage:
# create_executable(<name>
#                 SRCS <source_files...>
#                 DEPS <dependency_targets...>
#                 [USES_CHRONO]
# )
function(create_executable EXE_NAME)
    cmake_parse_arguments(
        "EXE" # prefix for parsed arguments
        "USES_CHRONO" # boolean arguments (flags)
        "" # single-value arguments
        "SRCS;DEPS" # multi-value arguments (lists)
        ${ARGN} # arguments to parse
    )

    # Add the executable itself
    add_executable(${EXE_NAME} ${EXE_SRCS})

    # Apply project root includes for the "src/..." include style
    add_project_root_includes_internal(${EXE_NAME})

    # Link dependencies
    if(EXE_DEPS)
        target_link_libraries(${EXE_NAME} PRIVATE ${EXE_DEPS})
    endif()

    # Link Chrono libraries if requested
    if(EXE_USES_CHRONO)
        target_link_libraries(${EXE_NAME} PRIVATE ${CHRONO_LIBRARIES})
        add_dependencies(${EXE_NAME} COPY_DLLS)
        if(CHRONO_DATA_DIR)
            target_compile_definitions(${EXE_NAME} PRIVATE CHRONO_DATA_DIR="${CHRONO_DATA_DIR}")
        endif()
        if(CHRONO_VEHICLE_DATA_DIR)
            target_compile_definitions(${EXE_NAME} PRIVATE CHRONO_VEHICLE_DATA_DIR="${CHRONO_VEHICLE_DATA_DIR}")
        endif()
    endif()

endfunction()

# Add the 'src' subdirectory. All your project's targets will be defined there.
add_subdirectory(src)