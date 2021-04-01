# Set source files
set(util_SOURCE_FILES
    file_io.h
    memory_management.h
    file_io.cpp
    memory_management.cpp
)

# Define the libraries
add_library(UTIL
            ${util_SOURCE_FILES}
)

add_compile_definitions(util_SOURCE_FILES PUBLIC STATE_SPACE_MATRIX_SIZE=2)
