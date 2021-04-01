# Set source files
set(elevator_saturation_SOURCE_FILES
    elevator_saturation.h
    elevator_saturation.cpp
)

set(PID_controller_SOURCE_FILES
    PID_controller.h
    PID_controller.cpp
)

set(safety_checks_SOURCE_FILES
    safety_checks.h
    safety_checks.cpp
)

# Define the libraries
ADD_LIBRARY(ELEVATOR_SATURATION
            ${elevator_saturation_SOURCE_FILES}
)

ADD_LIBRARY(PID_CONTROLLER
            ${PID_controller_SOURCE_FILES}
)

ADD_LIBRARY(SAFETY_CHECKS
            ${safety_checks_SOURCE_FILES}
)
