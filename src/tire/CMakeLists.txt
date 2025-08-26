create_executable(demo_salaani_model
    SRCS demo_salaani_model.cpp
    DEPS salaani_model
)

create_library(salaani_model STATIC
    SRCS salaani_model.cpp
    HDRS salaani_model.h
    DEPS
)