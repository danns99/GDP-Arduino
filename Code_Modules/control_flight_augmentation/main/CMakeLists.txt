# Set source files
set(main_arduino_SOURCE_FILES
    control.h
    control.cpp
)

# Define the libraries
ADD_LIBRARY(main_arduino
            ${main_arduino_SOURCE_FILES}
)
