cmake_minimum_required(VERSION 3.15)
project(Arduino_Code)
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -pedantic -Werror)


# Variables for directories
set(CODE_MODULES "Code_Modules/")
set(CONTROL_FLIGHT_AUGMENTATION "${CODE_MODULES}control_flight_augmentation/")
set(TESTS "Tests/")
set(CONTROL_MODEL_TESTING "${TESTS}Control_Model_Testing/source/")

# GLOB the Arduino code modules
# Control flight augmentation files
file(GLOB_RECURSE control_SOURCE_FILES
        "${CONTROL_FLIGHT_AUGMENTATION}control/*.h"
        "${CONTROL_FLIGHT_AUGMENTATION}control/*.cpp")

file(GLOB_RECURSE main_SOURCE_FILES
        "${CONTROL_FLIGHT_AUGMENTATION}main/*.h"
        "${CONTROL_FLIGHT_AUGMENTATION}main/*.cpp")

file(GLOB_RECURSE state_space_equations_SOURCE_FILES
        "${CONTROL_FLIGHT_AUGMENTATION}state_space_equations/*.h"
        "${CONTROL_FLIGHT_AUGMENTATION}state_space_equations/*.cpp")

file(GLOB_RECURSE util_SOURCE_FILES
        "${CONTROL_FLIGHT_AUGMENTATION}util/*.h"
        "${CONTROL_FLIGHT_AUGMENTATION}util/*.cpp")

# GLOB the test code sources
file(GLOB_RECURSE control_model_testing_main_SOURCE_FILES
        "${CONTROL_MODEL_TESTING}main/*.h"
        "${CONTROL_MODEL_TESTING}main/*.cpp")

file(GLOB_RECURSE control_model_testing_sim_SOURCE_FILES
        "${CONTROL_MODEL_TESTING}sim/*.h"
        "${CONTROL_MODEL_TESTING}sim/*.cpp")

file(GLOB_RECURSE control_model_testing_util_SOURCE_FILES
        "${CONTROL_MODEL_TESTING}util/*.h"
        "${CONTROL_MODEL_TESTING}util/*.cpp")

set(test_control_flight_augmentation_SOURCE_FILES
    ${control_model_testing_main_SOURCE_FILES}
    ${control_model_testing_sim_SOURCE_FILES}
    ${control_model_testing_util_SOURCE_FILES})

# Set sources for tests
# Control flight augmentation
set(tests_control_flight_augmentation_SOURCE_FILES
    ${test_control_flight_augmentation_SOURCE_FILES}
    ${control_SOURCE_FILES}
    ${state_space_equations_SOURCE_FILES})

# Test executables
add_executable(sim_runner_full_longitudinal_model
                ${tests_control_flight_augmentation_SOURCE_FILES})
add_executable(sim_runner_spo_model
                ${tests_control_flight_augmentation_SOURCE_FILES})

# Set preprocessor compile definitions
# Set STATE_SPACE_MATRIX_SIZE to:
# 4 for the full longitudinal model or
# 2 for the SPO approximation
target_compile_definitions(sim_runner_full_longitudinal_model PUBLIC STATE_SPACE_MATRIX_SIZE=4)
target_compile_definitions(sim_runner_spo_model PUBLIC STATE_SPACE_MATRIX_SIZE=2)

# Set the build locations for the executables
set_target_properties(sim_runner_spo_model PROPERTIES
                        ARCHIVE_OUTPUT_DIRECTORY "${TESTS}/Control_Model_Testing/lib"
                        LIBRARY_OUTPUT_DIRECTORY "${TESTS}/Control_Model_Testing/lib"
                        RUNTIME_OUTPUT_DIRECTORY "${TESTS}/Control_Model_Testing/bin")
set_target_properties(sim_runner_full_longitudinal_model PROPERTIES
                        ARCHIVE_OUTPUT_DIRECTORY "${TESTS}/Control_Model_Testing/lib"
                        LIBRARY_OUTPUT_DIRECTORY "${TESTS}/Control_Model_Testing/lib"
                        RUNTIME_OUTPUT_DIRECTORY "${TESTS}/Control_Model_Testing/bin")
