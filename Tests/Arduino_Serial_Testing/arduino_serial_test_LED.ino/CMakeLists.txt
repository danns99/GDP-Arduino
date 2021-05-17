cmake_minimum_required(VERSION 2.8)
### TOOLCHAIN SETUP AREA #################################################
# Set any variables used in the toolchain prior project() call. In that
# case they are already set and used.
##########################################################################

##################################################################################
# tools to be used for programming the AVR
##################################################################################
set(AVR_UPLOADTOOL avrdude)
set(AVR_PROGRAMMER usbasp)
set(AVR_UPLOADTOOL_PORT usb)

#### For Arduino bootloader:
#set(AVR_PROGRAMMER arduino)
#set(AVR_UPLOADTOOL_PORT /dev/tty.wchusbserial1420)
#set(AVR_UPLOADTOOL_BITRATE 57600)


# AVR and fuses needs to be set
set(AVR_MCU atmega328p)
set(AVR_H_FUSE 0xDA)
set(AVR_L_FUSE 0xFF)
set(AVR_EX_FUSE 0×05)
set(MCU_SPEED "16000000UL")

### printf support
#set(EXTRA_FLAGS "-Wl,-u,vfprintf -lprintf_min") 

### END TOOLCHAIN SETUP AREA #############################################
INCLUDE(generic-gcc-avr.cmake)


##########################################################################
# name your project
##########################################################################
project(avr_project_template)

##################################################################################
# add all sources and headers from ./src
##################################################################################
file(GLOB_RECURSE SRC_C_FILES ${PROJECT_SOURCE_DIR}/source/*.c)
file(GLOB_RECURSE SRC_CXX_FILES ${PROJECT_SOURCE_DIR}/source/*.cpp)
file(GLOB_RECURSE SRC_HEADERS ${PROJECT_SOURCE_DIR}/source/*.h)

##################################################################################
# add AVR executable
##################################################################################
add_avr_executable(
   ${PROJECT_NAME}
   ${SRC_C_FILES} ${SRC_CXX_FILES} ${SRC_HEADERS}
)

find_library(C_LIB c)
message(STATUS "avr-libc: ${C_LIB}")

avr_target_link_libraries(${PROJECT_NAME} ${C_LIB})


##################################################################################
# add AVR library
##################################################################################
#add_avr_library(
#   <library-name>
#   <sources> <headers>
#)

##################################################################################
# link library to executable
# NOTE: It needs to be the elf target.
##################################################################################
# target_link_libraries(<executable-name>-${AVR_MCU}.elf <library-name>-${AVR_MCU})
# OR easier








# cmake_minimum_required(VERSION 3.8)
# set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/Arduino-toolchain.cmake)
# project(Arduino_Serial_Test)
# # set(CMAKE_CXX_STANDARD 14)
# # add_compile_options(-Wall -Wextra -pedantic -Werror)

# set(SERIAL_TEST_LED_SRC "source/arduino_serial_test_LED.cpp")

# add_executable(arduino_serial_test SERIAL_TEST_LED_SRC)

# # Arduino.h is included in hello_world.cpp, so link with Arduino core
# target_link_arduino_libraries(arduino_serial_test PRIVATE core)

# # This is needed for the generation of HEX binary and uploading it
# target_enable_arduino_upload(arduino_serial_test SERIAL_TEST_LED_SRC)


# # Call a framework utility function, passing it information about the hardware board that will
# # be used - This function returns a structure known only to the framework
# get_board_id(board_id uno)

# set(SERIAL_TEST_LED_SKETCH "source/arduino_serial_test_LED.cpp")

# # Create an executable suitable for the Arduino firmware using CMake-style target-creation
# add_arduino_executable(serial_test ${board_id} arduino_serial_test.cpp)
# # Upload the created target through a connected Serial Port (Where your board is connected to)
# #upload_arduino_target(serial_test_LED "${board_id}" COM3)


# set(ARDUINO_DEFAULT_BOARD uno) # Default Board ID, when not specified
# set(ARDUINO_DEFAULT_PORT /dev/ttyUSB0) # Default Port, when not specified

# set(SERIAL_TEST_LED_SKETCH "source/arduino_serial_test_LED.cpp")

# set(serial_test_LED_SRCS "${SERIAL_TEST_LED_SKETCH}")
# #set(serial_test_LED_PORT  /dev/ttyACM)
# #set(serial_test_LED_SERIAL picocom @SERIAL_PORT@)
# set(serial_test_LED_BOARD uno)

# #generate_arduino_firmware(serial_test_LED)

# generate_arduino_firmware(arduino_serial_test_LED
#     SRCS "source/arduino_serial_test_LED.cpp"
#     BOARD uno)
