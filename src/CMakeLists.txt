add_subdirectory(driveline)
add_subdirectory(simulation_interface)
add_subdirectory(steering)
add_subdirectory(utils)
add_subdirectory(vehicle)
add_subdirectory(suspension_test_rig)
add_subdirectory(python)

create_executable(cosim_forces
    SRCS cosim_forces.cpp
    DEPS
    USES_CHRONO
)
# add_executable(cosim_forces cosim_forces.cpp)
# target_link_libraries(cosim_forces PRIVATE
#     ${CHRONO_LIBRARIES}
# )

create_executable(cosim_ford_expedition_2003
    SRCS cosim_ford_expedition_2003.cpp
    DEPS
    USES_CHRONO
)
# add_executable(cosim_ford_expedition_2003 cosim_ford_expedition_2003.cpp)
# target_link_libraries(cosim_ford_expedition_2003 PRIVATE
#     ${CHRONO_LIBRARIES}
# )

create_executable(force_rack_pinion_veh_example 
    SRCS force_rack_pinion_veh_example.cpp
    DEPS simulation_interface utils steering vehicle
    USES_CHRONO
)

# add_executable(force_rack_pinion_veh_example force_rack_pinion_veh_example.cpp)
# target_link_libraries(force_rack_pinion_veh_example PRIVATE
#     ${CHRONO_LIBRARIES}
#     simulation_interface
#     utils
#     steering
#     vehicle
# )
