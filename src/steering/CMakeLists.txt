# add_library(steering STATIC ChRackPinionForce.cpp)
# set_target_properties(steering PROPERTIES
#     COMPILE_FLAGS "${CHRONO_CXX_FLAGS}"
#     COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
#     LINK_FLAGS "${CHRONO_LINKER_FLAGS}"
#     # INCLUDE_DIRECTORIES "${CHRONO_INCLUDE_DIRS}"
#     LINK_LIBRARIES "${CHRONO_LIBRARIES}"
#     COMPILE_FEATURES "cxx_std_17"
# )
# target_include_directories(steering 
#     PUBLIC 
#         ${CMAKE_SOURCE_DIR}/src/steering
#         ./
#         ${CHRONO_INCLUDE_DIRS}
# )
# target_compile_definitions(steering PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"")
# target_compile_options(steering PUBLIC ${CHRONO_CXX_FLAGS})
# target_link_options(steering PUBLIC ${CHRONO_LINKER_FLAGS})
# target_compile_features(steering PUBLIC "cxx_std_17")
# target_link_libraries(steering ${CHRONO_LIBRARIES})