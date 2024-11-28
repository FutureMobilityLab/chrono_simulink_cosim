# add_library(utils STATIC utils.cpp)
# set_target_properties(utils PROPERTIES
#     COMPILE_FLAGS "${CHRONO_CXX_FLAGS}"
#     COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
#     LINK_FLAGS "${CHRONO_LINKER_FLAGS}"
#     # LINK_LIBRARIES "steering ${CHRONO_LIBRARIES}"
#     COMPILE_FEATURES "cxx_std_17"
# )
# target_link_libraries(utils steering ${CHRONO_LIBRARIES})
# target_include_directories(utils 
#     PUBLIC 
#         ${CMAKE_SOURCE_DIR}/src/steering
#         ./
#         ${CHRONO_INCLUDE_DIRS}
# )