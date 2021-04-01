# Set source files
set(main_SOURCE_FILES
    control.h
    control.cpp
)

# Define the libraries
add_library(CONTROL
            ${main_SOURCE_FILES}
)

add_compile_definitions(main_SOURCE_FILES PUBLIC STATE_SPACE_MATRIX_SIZE=2)
