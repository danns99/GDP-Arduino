# Set source files
set(backward_euler_analytic_SOURCE_FILES
    backward_euler_analytic.h
    backward_euler_analytic.cpp
)

set(backward_euler_newton_SOURCE_FILES
    backward_euler_newton.h
    backward_euler_newton.cpp
)


# Define the libraries
ADD_LIBRARY(BACKWARD_EULER_ANALYTIC
            ${backward_euler_analytic_SOURCE_FILES}
)

ADD_LIBRARY(BACKWARD_EULER_NEWTON
            ${backward_euler_newton_SOURCE_FILES}
)
