# Set source files
set(state_space_equations_SOURCE_FILES
    state_space_equation.h
    state_space_equation.cpp
)

# Define the libraries
ADD_LIBRARY(STATE_SPACE_EQUATIONS
            ${state_space_equations_SOURCE_FILES}
)
