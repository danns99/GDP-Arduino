cmake_minimum_required(VERSION 3.15)
project(Control_Model_testing)
set(CMAKE_CXX_STANDARD 14)

# Set to 4 for the full longitudinal model or 2 for the SPO approximation
add_compile_definitions(STATE_SPACE_MATRIX_SIZE=4)

file(GLOB_RECURSE util_SOURCE_FILES
        "util/*.h" "util/*.cpp")

file(GLOB_RECURSE control_model_SOURCE_FILES
        "Control_Model/*.h" "Control_Model/*.cpp")

file(GLOB_RECURSE controller_SOURCE_FILES
        "controller/*.h" "controller/*.cpp")

file(GLOB_RECURSE control_loop_SOURCE_FILES
        "control_loop/*.h" "control_loop/*.cpp")

file(GLOB_RECURSE state_space_equations_SOURCE_FILES
        "state_space_equations/*.h" "state_space_equations/*.cpp")

file(GLOB_RECURSE sim_SOURCE_FILES
        "sim/*.h" "sim/*.cpp")

add_executable(sim_runner sim_runner.cpp ${util_SOURCE_FILES}
                ${control_model_SOURCE_FILES} ${controller_SOURCE_FILES}
                ${control_loop_SOURCE_FILES}
                ${state_space_equations_SOURCE_FILES} ${sim_SOURCE_FILES})
