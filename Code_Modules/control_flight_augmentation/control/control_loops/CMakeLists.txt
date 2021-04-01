# Set source files
set(spo_control_loop_SOURCE_FILES
    control_loop_spo_model.h
    control_loop_spo_model.cpp
)

set(full_longitudinal_control_loop_SOURCE_FILES
    control_loop_spo_model.h
    control_loop_spo_model.cpp
)

# Define the libraries
ADD_LIBRARY(SPO_CONTROL_LOOP
            ${spo_control_loop_SOURCE_FILES}
)

ADD_LIBRARY(FULL_LONGITUDINAL_CONTROL_LOOP
            ${full_longitudinal_control_loop_SOURCE_FILES}
)
