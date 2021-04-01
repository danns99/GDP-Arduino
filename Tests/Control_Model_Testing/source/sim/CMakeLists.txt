# Set source files
set(sim_SOURCE_FILES
    sim.h
    sim.cpp
)

# Define the libraries
ADD_LIBRARY(SIM
            ${sim_SOURCE_FILES}
)

add_compile_definitions(sim_SOURCE_FILES PUBLIC STATE_SPACE_MATRIX_SIZE=2)
