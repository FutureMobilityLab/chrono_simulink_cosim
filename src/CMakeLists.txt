add_subdirectory(driveline)
add_subdirectory(simulation_interface)
add_subdirectory(steering)
add_subdirectory(tire)
add_subdirectory(utils)
add_subdirectory(vehicle)
add_subdirectory(suspension_test_rig)

create_executable(cosim_forces
    SRCS cosim_forces.cpp
    DEPS
    USES_CHRONO
)

create_executable(cosim_ford_expedition_2003
    SRCS cosim_ford_expedition_2003.cpp
    DEPS
    USES_CHRONO
)

create_executable(force_rack_pinion_veh_example 
    SRCS force_rack_pinion_veh_example.cpp
    DEPS simulation_interface
    USES_CHRONO
)

create_executable(tire_test_rig
    SRCS TireTestRig.cpp
    DEPS utils
    USES_CHRONO
)
