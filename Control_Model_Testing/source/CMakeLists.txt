cmake_minimum_required(VERSION 3.15)
project(Control_Model_testing)
set(CMAKE_CXX_STANDARD 14)

file(GLOB_RECURSE util_SOURCE_FILES
        "util/*.h" "util/*.cpp")

add_executable(Control_Model Control_Model.cpp ${util_SOURCE_FILES})
#add_executable(Control_Model Control_Model.cpp)
