cmake_minimum_required(VERSION 3.15)
project(Control_Model_testing)
set(CMAKE_CXX_STANDARD 14)

file(GLOB_RECURSE util_SOURCE_FILES
        "util/*.h" "util/*.cpp")

file(GLOB_RECURSE controller_SOURCE_FILES
        "controller/*.h" "controller/*.cpp")

file(GLOB_RECURSE state_space_equations_SOURCE_FILES
        "state_space_equations/*.h" "state_space_equations/*.cpp")

add_executable(Control_Model Control_Model.cpp ${util_SOURCE_FILES}
                ${controller_SOURCE_FILES} ${state_space_equations_SOURCE_FILES})
