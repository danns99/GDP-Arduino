# Set source files
set(util_arduino_SOURCE_FILES
    file_io.h
    file_io.cpp
)

# Define the libraries
ADD_LIBRARY(UTIL_ARDUINO
            ${util_arduino_SOURCE_FILES}
)
