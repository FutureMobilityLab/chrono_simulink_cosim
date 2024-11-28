# add_subdirectory(steering)
# add_subdirectory(utils)

# # Get a list of all .cpp files in the 'src' directory
# file(GLOB SOURCES "*.cpp")

# # Create an executable for each source file
# foreach(SOURCE_FILE ${SOURCES})
#     message(STATUS "Source files for executables: ${SOURCE_FILE}")
#     # message(STATUS "Chrono libraries: ${CHRONO_LIBRARIES}")
#     get_filename_component(EXECUTABLE_NAME ${SOURCE_FILE} NAME_WE)
#     add_executable(${EXECUTABLE_NAME} ${SOURCE_FILE})
#     target_compile_definitions(${EXECUTABLE_NAME} PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"")
#     target_compile_options(${EXECUTABLE_NAME} PUBLIC ${CHRONO_CXX_FLAGS})
#     target_link_options(${EXECUTABLE_NAME} PUBLIC ${CHRONO_LINKER_FLAGS})
#     target_compile_features(${EXECUTABLE_NAME} PUBLIC "cxx_std_17")
#     target_link_libraries(${EXECUTABLE_NAME} PUBLIC ${CHRONO_LIBRARIES})
# endforeach()
