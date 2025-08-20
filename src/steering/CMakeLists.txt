create_library(steering STATIC
    SRCS ChRackPinionForce.cpp RackPinionForce.cpp
    HDRS ChRackPinionForce.h RackPinionForce.h
    DEPS
    USES_CHRONO
)