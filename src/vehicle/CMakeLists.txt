create_library(vehicle STATIC
    SRCS WheeledVehicleForce.cpp
    HDRS WheeledVehicleForce.h
    DEPS steering utils
    USES_CHRONO
)