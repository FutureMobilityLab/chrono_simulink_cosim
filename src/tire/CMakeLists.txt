create_executable(demo_salaani_model
    SRCS demo_salaani_model.cpp
    DEPS salaani_model
)

create_library(salaani_model STATIC
    SRCS salaani_model.cpp
    HDRS salaani_model.h
    DEPS
)

create_library(ch_salaani_tire STATIC
    SRCS ChSalaaniTire.cpp
    HDRS ChSalaaniTire.h
    DEPS salaani_model utils
    USES_CHRONO
)

create_library(salaani_tire STATIC
    SRCS SalaaniTire.cpp
    HDRS SalaaniTire.h
    DEPS ch_salaani_tire
    USES_CHRONO
)