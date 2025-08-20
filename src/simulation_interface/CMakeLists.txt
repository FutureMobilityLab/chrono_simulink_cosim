create_library(simulation_interface STATIC
    SRCS SimulationInterface.cpp
    HDRS SimulationInterface.h
    DEPS vehicle utils steering
    USES_CHRONO
)

create_library(simulation_interface_c_api MODULE
    SRCS SimulationInterface_c_api.cpp
    HDRS SimulationInterface_c_api.h
    DEPS simulation_interface
)
set_target_properties(simulation_interface_c_api PROPERTIES
    WINDOWS_EXPORT_ALL_SYMBOLS ON
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}"
)
add_dependencies(simulation_interface_c_api COPY_DLLS)