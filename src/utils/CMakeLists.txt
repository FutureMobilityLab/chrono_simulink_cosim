create_library(utils STATIC
    SRCS utils.cpp
    HDRS utils.h
    DEPS steering salaani_tire
    USES_CHRONO
)