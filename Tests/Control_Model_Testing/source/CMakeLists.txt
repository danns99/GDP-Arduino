# cmake_minimum_required(VERSION 3.15)
# project(Control_Model_testing)
# set(CMAKE_CXX_STANDARD 14)

# add_compile_options(-Wall -Wextra -pedantic -Werror)


# Group together source code files
# file(GLOB_RECURSE file_io_SOURCE_FILES
#         "util/file_io.h"
#         "util/file_io.cpp")

# file(GLOB_RECURSE util_SOURCE_FILES
#         "././Code_Modules/control_flight_augmentation/util/memory_management.h"
#         "././Code_Modules/control_flight_augmentation/util/memory_management.cpp")

# file(GLOB_RECURSE control_SOURCE_FILES
#         "././Code_Modules/control_flight_augmentation/control/*.h"
#         "././Code_Modules/control_flight_augmentation/control/*.cpp")

# file(GLOB_RECURSE control_spo_SOURCE_FILES
#         "././Code_Modules/control_flight_augmentation/control/control_loops/control_loop_spo_model.h"
#         "././Code_Modules/control_flight_augmentation/control/control_loops/control_loop_spo_model.cpp")

# file(GLOB_RECURSE control_full_longitudinal_model_SOURCE_FILES
#         "././Code_Modules/control_flight_augmentation/control/control_loops/control_full_longitudinal_model.h"
#         "././Code_Modules/control_flight_augmentation/control/control_loops/control_full_longitudinal_model.cpp")

# file(GLOB_RECURSE state_space_equations_SOURCE_FILES
#         "././Code_Modules/control_flight_augmentation/state_space_equations/*.h"
#         "././Code_Modules/control_flight_augmentation/state_space_equations/*.cpp")

# file(GLOB_RECURSE main_SOURCE_FILES
#         "main/*.h"
#         "main/*.cpp")

# file(GLOB_RECURSE sim_SOURCE_FILES
#         "sim/*.h" "sim/*.cpp")

# Include required libraries
# link_directories(${MyProject_SOURCE_DIR}/Tests/Control_Model_Testing/main)
# link_directories(${MyProject_SOURCE_DIR}/Tests/Control_Model_Testing/sim)
# link_directories(${MyProject_SOURCE_DIR}/Tests/Control_Model_Testing/util)
# link_directories(${MyProject_SOURCE_DIR}/Code_Modules/Control_Model_Testing/control_flight_augmentation/)

# Add subdirectories
add_subdirectory(main)
add_subdirectory(sim)
add_subdirectory(util)
#add_subdirectory(control_flight_augmentation/control)

# Executables
# add_executable(sim_runner_full_longitudinal_model  ${file_io_SOURCE_FILES} ${util_SOURCE_FILES}
#                 ${control_SOURCE_FILES} ${control_full_longitudinal_model_SOURCE_FILES}
#                 ${state_space_equations_SOURCE_FILES} ${sim_SOURCE_FILES}
#                 ${main_SOURCE_FILES})

# add_executable(sim_runner_SPO_model  ${file_io_SOURCE_FILES} ${util_SOURCE_FILES}
#                 ${control_SOURCE_FILES} ${control_spo_SOURCE_FILES}
#                 ${state_space_equations_SOURCE_FILES} ${sim_SOURCE_FILES}
#                 ${main_SOURCE_FILES})

#add_executable(sim_runner_SPO_model  main/control.cpp)

# Set preprocessor compile definitions
# Set STATE_SPACE_MATRIX_SIZE to:
# 4 for the full longitudinal model or
# 2 for the SPO approximation
#target_compile_definitions(sim_runner_full_longitudinal_model PUBLIC STATE_SPACE_MATRIX_SIZE=4)

# Link Libraries
#target_link_libraries(sim_runner_full_longitudinal_model main sim util)
#target_link_libraries(sim_runner_SPO_model SIM UTIL SPO_CONTROL_LOOP)

#target_compile_definitions(sim_runner_SPO_model PUBLIC STATE_SPACE_MATRIX_SIZE=2)
