create_executable(demo_salaani_model
    SRCS demo_salaani_model.cpp
    DEPS salaani_model
)

create_executable(demo_stiremod
    SRCS demo_stiremod.cpp
    DEPS stiremod
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

create_library(stiremod STATIC
    SRCS stiremod.cpp
    HDRS stiremod.h
    DEPS
)

create_library(ch_stiremod_tire STATIC
    SRCS ChStiremodTire.cpp
    HDRS ChStiremodTire.h
    DEPS stiremod utils
    USES_CHRONO
)

create_library(stiremod_tire STATIC
    SRCS StiremodTire.cpp
    HDRS StiremodTire.h
    DEPS ch_stiremod_tire
    USES_CHRONO
)