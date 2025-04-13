#set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
#--------------------------------------------------------------
# 
# Example of CMake configuration file to build an external 
# project depending on Chrono and on optional Chrono modules.
# 
# This minimal sample project can be used as a template for a
# user project.  Modify sections 1, 2, and 3 below as appropriate.
# 
#--------------------------------------------------------------
 

cmake_minimum_required(VERSION 3.14)

#--------------------------------------------------------------
# === 1 === 
# Modify the project name if you want: 
#--------------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
project(chrono_simulink_cosim)

#--------------------------------------------------------------
# === 2 ===
# Find the Chrono package and any REQUIRED or OPTIONAL modules
# by invoking the find_package function in CONFIG mode:
#    find_package(Chrono
#                 COMPONENTS req_module1 req_module1 ...
#                 OPTIONAL_COMPONENTS opt_module1 opt_module2 ...
#                 CONFIG)
# The following Chrono modules can be requested (their names
# are case insensitive): Cascade, Cosimulation, FEA, Irrlicht,
# Matlab, Parallel, Postprocess, Python, Vehicle.
# 
# Note that you will have to set the variable Chrono_DIR to 
# specify the location of the ChronoConfig.cmake script, if
# it is not in its default install location.
# Chrono_DIR can be either a Chrono build tree or a Chrono install tree.
# 
# The following variables are set and can be used further down:
# CHRONO_FOUND
#   set to true if Chrono and all required components were found
# CHRONO_C_FLAGS
# CHRONO_CXX_FLAGS
#   C and C++ compilation flags
# CHRONO_INCLUDE_DIRS
#   additional paths for included headers
# CHRONO_LIBRARIES
#   list of required libraries (with full path)
# CHRONO_LINKER_FLAGS
#   additional linker flags
# CHRONO_DLLS
#   list of all DLL dependencies (with full path)
# CHRONO_DATA_DIR
#   path to the Chrono data make_directory
# 
# In addition, for each requested component [COMPONENT], the
# following variable is set to true (ON) or false (OFF):
# CHRONO_[COMPONENT]_FOUND
# 
# In this example, we only request the Irrlicht module (required)
# and, for demonstration purposes, the PardisoMKL module (optional)
#--------------------------------------------------------------

find_package(pybind11 REQUIRED)
# add_subdirectory(${pybind11_DIR})
# include_directories(${pybind11_DIR}/include)

LIST(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/../Chrono/lib")
find_package(Chrono
             COMPONENTS Vehicle Irrlicht Cosimulation
             OPTIONAL_COMPONENTS PardisoMKL Postprocess
             CONFIG)

# Get GTest. This is much easier than having the user install GTest on windows.
# The URL is a the hash to a commit for the googletest version. It is a good
# idea to keep this updated.
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# add_subdirectory(src)
# add_subdirectory(test)

#--------------------------------------------------------------
# Return now if Chrono or a required component was not found.
#--------------------------------------------------------------

if (NOT Chrono_FOUND)
  message("Could not find Chrono or one of its required modules")
  return()
endif()

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

include_directories(${CHRONO_INCLUDE_DIRS})

#-----------------------------------------------------------------------------
# Fix for VS 2017 15.8 and newer to handle alignment specification with Eigen
#-----------------------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  if(MSVC AND ${MSVC_VERSION} GREATER_EQUAL 1915)
    add_definitions( "-D_ENABLE_EXTENDED_ALIGNED_STORAGE" )
  endif()
endif()

#--------------------------------------------------------------
# Tweaks to disable some warnings with MSVC
#--------------------------------------------------------------
if(MSVC)
    add_definitions("-D_CRT_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    add_definitions("-D_SCL_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    add_definitions( "-DNOMINMAX" )                # do not use MSVC's min/max macros
    add_compile_options(/wd4275)                   # disable warnings triggered by Irrlicht
    add_compile_options(/wd4251)                   # class needs to have dll-interface
endif()

#--------------------------------------------------------------
# === 3 ===
# Add the executable from your project and specify all C++ 
# files in your project. 
#--------------------------------------------------------------

add_compile_definitions(CH_API_COMPILE_VEHICLE)

add_library(steering 
  src/steering/ChRackPinionForce.cpp  src/steering/ChRackPinionForce.h
  src/steering/RackPinionForce.cpp    src/steering/RackPinionForce.h
)
set_target_properties(steering PROPERTIES 
	COMPILE_FLAGS "${CHRONO_CXX_FLAGS}"
	COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
	LINK_FLAGS "${CHRONO_LINKER_FLAGS}")
target_include_directories(steering PUBLIC 
    ${CMAKE_SOURCE_DIR}
    ${CHRONO_INCLUDE_DIRS}
)
target_link_libraries(steering ${CHRONO_LIBRARIES})

add_library(utils 
  src/utils/utils.cpp
)
set_target_properties(utils PROPERTIES 
	COMPILE_FLAGS "${CHRONO_CXX_FLAGS}"
	COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
	LINK_FLAGS "${CHRONO_LINKER_FLAGS}"
)
target_include_directories(utils PUBLIC 
    ${CMAKE_SOURCE_DIR}
    ${CHRONO_INCLUDE_DIRS}
)
target_link_libraries(utils ${CHRONO_LIBRARIES} steering)

add_library(vehicle
  src/vehicle/WheeledVehicleForce.cpp src/vehicle/WheeledVehicleForce.h
)
set_target_properties(vehicle PROPERTIES 
	COMPILE_FLAGS "${CHRONO_CXX_FLAGS}"
	COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
	LINK_FLAGS "${CHRONO_LINKER_FLAGS}"
)
target_include_directories(vehicle PUBLIC 
    ${CMAKE_SOURCE_DIR}
    ${CHRONO_INCLUDE_DIRS}
)
target_link_libraries(vehicle ${CHRONO_LIBRARIES} steering utils)

add_library(suspension_test_rig
  src/suspension_test_rig/SuspensionTestRig.cpp src/suspension_test_rig/SuspensionTestRig.h
)
set_target_properties(suspension_test_rig PROPERTIES 
	COMPILE_FLAGS "${CHRONO_CXX_FLAGS}"
	COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
	LINK_FLAGS "${CHRONO_LINKER_FLAGS}"
)
target_include_directories(suspension_test_rig PUBLIC 
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
    ${CHRONO_INCLUDE_DIRS}
)
target_link_libraries(suspension_test_rig ${CHRONO_LIBRARIES})

add_library(
  simulation_interface SHARED
  src/simulation_interface/SimulationInterface.cpp
  src/simulation_interface/SimulationInterface.h  
)
# target_compile_options(simulation_interface PUBLIC ${CHRONO_CXX_FLAGS})
set_target_properties(simulation_interface PROPERTIES 
	COMPILE_FLAGS "${CHRONO_CXX_FLAGS}"
	COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
	LINK_FLAGS "${CHRONO_LINKER_FLAGS}"
	WINDOWS_EXPORT_ALL_SYMBOLS ON
)
target_include_directories(simulation_interface PUBLIC
  ${CHRONO_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
)
# target_link_options(simulation_interface PUBLIC ${CHRONO_LINKER_FLAGS})
target_link_libraries(simulation_interface PUBLIC
  ${CHRONO_LIBRARIES}
  vehicle
  # suspension_test_rig
  utils
  steering
)

add_library(
  simulation_interface_c_api MODULE
  src/simulation_interface/SimulationInterface_c_api.cpp
  src/simulation_interface/SimulationInterface_c_api.h
)
set_target_properties(simulation_interface_c_api PROPERTIES 
	COMPILE_FLAGS "${CHRONO_CXX_FLAGS}"
	COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
	LINK_FLAGS "${CHRONO_LINKER_FLAGS}"
)
target_include_directories(simulation_interface_c_api PUBLIC
  ${CHRONO_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
)
target_link_libraries(simulation_interface_c_api 
  ${CHRONO_LIBRARIES}
  # vehicle
  simulation_interface
)

# Get a list of all .cpp files in the 'src' directory
file(GLOB SOURCES "src/*.cpp")

# Create an executable for each source file
foreach(SOURCE_FILE ${SOURCES})
    # Extract the filename without extension
    get_filename_component(EXECUTABLE_NAME ${SOURCE_FILE} NAME_WE)

    # Add the executable
    add_executable(${EXECUTABLE_NAME} ${SOURCE_FILE})

#--------------------------------------------------------------
# Set properties for your executable target
# 
# Note that here we define a macro CHRONO_DATA_DIR which will
# contain the path to the Chrono data directory, either in its
# source tree (if using a build version of Chrono), or in its
# install tree (if using an installed version of Chrono).
#--------------------------------------------------------------

  target_compile_definitions(${EXECUTABLE_NAME} PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"")
  target_compile_options(${EXECUTABLE_NAME} PUBLIC ${CHRONO_CXX_FLAGS})
  target_link_options(${EXECUTABLE_NAME} PUBLIC ${CHRONO_LINKER_FLAGS})
  target_compile_features(${EXECUTABLE_NAME} PUBLIC "cxx_std_17")

#--------------------------------------------------------------
# Link to Chrono libraries and dependency libraries
#--------------------------------------------------------------

  add_dependencies(${EXECUTABLE_NAME}
    vehicle
    simulation_interface
    suspension_test_rig
    utils
    steering
  )
  
  target_link_libraries(${EXECUTABLE_NAME} PUBLIC
    ${CHRONO_LIBRARIES} 
    vehicle 
    simulation_interface 
    suspension_test_rig
    utils
    steering
  )

endforeach()

# set_target_properties(vehicle_interface PROPERTIES PREFIX "" SUFFIX ".pyd")
# add_library(vehicle_interface MODULE src/python/vehicle_interface.cpp)
# target_link_libraries(vehicle_interface PRIVATE
#     steering utils vehicle ${CHRONO_LIBRARIES} pybind11::module
# )
# target_include_directories(vehicle_interface PUBLIC
#     ${CHRONO_INCLUDE_DIRS} 
#     ${pybind11_INCLUDE_DIRS}
# )
# set_target_properties(vehicle_interface PROPERTIES PREFIX "" SUFFIX ".pyd")
# # target_link_libraries(vehicle_interface ${CHRONO_LIBRARIES} steering utils vehicle)
# # target_include_directories(vehicle_interface PUBLIC 
# #     ${CHRONO_INCLUDE_DIRS}
# #     ${pybind11_DIR}/include
# # )

# # TEST INTERFACE (DELETE THIS AFTER PROVING PYBIND WORKS)
# add_library(test_interface MODULE src/python/test_interface.cpp)

# target_link_libraries(test_interface PRIVATE
#     pybind11::module
#     ${CHRONO_LIBRARIES}  # Add Chrono libraries just in case
# )

# target_include_directories(test_interface PUBLIC
#     ${pybind11_INCLUDE_DIRS}
#     ${CHRONO_INCLUDE_DIRS}  # Add Chrono includes
# )

# # Set Python-specific properties for the module
# set_target_properties(test_interface PROPERTIES
#     PREFIX ""
#     SUFFIX ".pyd"
#     LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
#     LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}"
#     LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}"
#     RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
#     RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}"
#     RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}"
#     VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
# )

# # Copy Python DLL to the build directory on Windows
# if(WIN32)
#     add_custom_command(TARGET test_interface POST_BUILD
#         COMMAND ${CMAKE_COMMAND} -E copy_if_different
#             "$<TARGET_FILE:test_interface>"
#             "${CMAKE_BINARY_DIR}/$<TARGET_FILE_NAME:test_interface>"
#     )
    
#     # Copy Visual C++ Runtime DLLs if using MSVC
#     if(MSVC)
#         include(InstallRequiredSystemLibraries)
#         add_custom_command(TARGET test_interface POST_BUILD
#             COMMAND ${CMAKE_COMMAND} -E copy_if_different
#                 ${CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS}
#                 "${CMAKE_BINARY_DIR}"
#         )
#     endif()
# endif()

# # Ensure the output directory exists
# file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR})

#--------------------------------------------------------------
# === 4 (OPTIONAL) ===
# 
# Optionally, add a custom command for copying all Chrono and
# dependency DLLs to the appropriate binary output folder.
# This function has effect only on Windows.
# 
# Note that you must first set EXECUTABLE_OUTPUT_PATH
# (this can simply be ${CMAKE_BINARY_DIR}, like in this example)
#--------------------------------------------------------------

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
add_DLL_copy_command("${CHRONO_DLLS}")

# set(CMAKE_BUILD_DIR ${PROJECT_SOURCE_DIR}/build)
# set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BUILD_DIR}/bin)
# set(LIBRARY_OUTPUT_PATH ${CMAKE_BUILD_DIR}/lib)