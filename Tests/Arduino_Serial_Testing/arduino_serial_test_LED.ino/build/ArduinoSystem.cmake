# Copyright (c) 2020 Arduino CMake Toolchain

set(ARDUINO_INSTALL_PATH "C:/Program Files (x86)/Arduino")
set(ARDUINO_PACKAGE_PATH "C:/Users/akern/AppData/Local/Arduino15")
set(ARDUINO_SKETCHBOOK_PATH "D:/Andrew/Documents/Arduino")

set(ARDUINO "108012")
set("ARDUINO_ARCH_AVR" TRUE)
set("ARDUINO_AVR_UNO" TRUE)

set(ARDUINO_BOARD "AVR_UNO")
set(ARDUINO_BOARD_IDENTIFIER "avr.uno")
set(ARDUINO_BOARD_NAME "Arduino Uno")
set(ARDUINO_BOARD_BUILD_ARCH "AVR")
set(ARDUINO_BOARD_RUNTIME_PLATFORM_PATH "C:/Program Files (x86)/Arduino/hardware/arduino/avr")
set(ARDUINO_CORE_SPECIFIC_PLATFORM_PATH "")
set(ARDUINO_BOARD_BUILD_CORE_PATH "C:/Program Files (x86)/Arduino/hardware/arduino/avr/cores/arduino")
set(ARDUINO_BOARD_BUILD_VARIANT_PATH "C:/Program Files (x86)/Arduino/hardware/arduino/avr/variants/standard")
set(ARDUINO_BOARD_HOST_NAME "windows")

set(ARDUINO_BOARD_UPLOAD_TOOL avrdude)
set(ARDUINO_BOARD_PROGRAM_TOOL )
set(ARDUINO_BOARD_BOOTLOADER_TOOL avrdude)

set(ARDUINO_PROGRAMMER_ID "")
set(ARDUINO_PROGRAMMER_NAME "")

set(ARDUINO_RULE_NAMES_LIST "recipe.c.o.pattern;recipe.cpp.o.pattern;recipe.S.o.pattern;recipe.ar.pattern;recipe.c.combine.pattern;recipe.objcopy.eep.pattern;recipe.objcopy.hex.pattern;recipe.size.pattern;tools.avrdude.upload.network_pattern;tools.avrdude.upload.pattern;tools.avrdude.program.pattern;tools.avrdude.erase.pattern;tools.avrdude.bootloader.pattern;tools.avrdude_remote.upload.pattern")
set("ARDUINO_RULE_recipe.c.o.pattern" "\"C:/Program Files (x86)/Arduino/hardware/tools/avr/bin/avr-gcc\" -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=108012 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   {includes} \"{source_file}\" -o \"{object_file}\"")
set("ARDUINO_RULE_recipe.cpp.o.pattern" "\"C:/Program Files (x86)/Arduino/hardware/tools/avr/bin/avr-g++\" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=108012 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   {includes} \"{source_file}\" -o \"{object_file}\"")
set("ARDUINO_RULE_recipe.S.o.pattern" "\"C:/Program Files (x86)/Arduino/hardware/tools/avr/bin/avr-gcc\" -c -g -x assembler-with-cpp -flto -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=108012 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   {includes} \"{source_file}\" -o \"{object_file}\"")
set("ARDUINO_RULE_recipe.ar.pattern" "\"C:/Program Files (x86)/Arduino/hardware/tools/avr/bin/avr-gcc-ar\" rcs  \"{build.path}/{archive_file}\" \"{object_file}\"")
set("ARDUINO_RULE_recipe.c.combine.pattern" "\"C:/Program Files (x86)/Arduino/hardware/tools/avr/bin/avr-gcc\" -w -Os -g -flto -fuse-linker-plugin -Wl,--gc-sections -mmcu=atmega328p  -o \"{build.path}/{build.project_name}.elf\" {object_files} \"{build.path}/{archive_file}\" \"-L{build.path}\" -lm")
set("ARDUINO_RULE_recipe.objcopy.eep.pattern" "\"C:/Program Files (x86)/Arduino/hardware/tools/avr/bin/avr-objcopy\" -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0  \"{build.path}/{build.project_name}.elf\" \"{build.path}/{build.project_name}.eep\"")
set("ARDUINO_RULE_recipe.objcopy.hex.pattern" "\"C:/Program Files (x86)/Arduino/hardware/tools/avr/bin/avr-objcopy\" -O ihex -R .eeprom  \"{build.path}/{build.project_name}.elf\" \"{build.path}/{build.project_name}.hex\"")
set("ARDUINO_RULE_recipe.size.pattern" "\"C:/Program Files (x86)/Arduino/hardware/tools/avr/bin/avr-size\" -A \"{build.path}/{build.project_name}.elf\"")
set("ARDUINO_RULE_tools.avrdude.upload.network_pattern" "\"C:/Program Files (x86)/Arduino/hardware/tools/avr/bin/arduinoOTA\" -address {serial.port} -port {upload.network.port} -sketch \"{build.path}/{build.project_name}.hex\" -upload {upload.network.endpoint_upload} -sync {upload.network.endpoint_sync} -reset {upload.network.endpoint_reset} -sync_exp {upload.network.sync_return}")
set("ARDUINO_RULE_tools.avrdude.upload.pattern" "\"C:/Program Files (x86)/Arduino/hardware/tools/avr/bin/avrdude\" \"-CC:/Program Files (x86)/Arduino/hardware/tools/avr/etc/avrdude.conf\" -v  -patmega328p -carduino \"-P{serial.port}\" -b115200 -D \"-Uflash:w:{build.path}/{build.project_name}.hex:i\"")
set("ARDUINO_RULE_tools.avrdude.program.pattern" "\"C:/Program Files (x86)/Arduino/hardware/tools/avr/bin/avrdude\" \"-CC:/Program Files (x86)/Arduino/hardware/tools/avr/etc/avrdude.conf\" -v  -patmega328p -c{protocol} {program.extra_params} \"-Uflash:w:{build.path}/{build.project_name}.hex:i\"")
set("ARDUINO_RULE_tools.avrdude.erase.pattern" "\"C:/Program Files (x86)/Arduino/hardware/tools/avr/bin/avrdude\" \"-CC:/Program Files (x86)/Arduino/hardware/tools/avr/etc/avrdude.conf\" -v -patmega328p -c{protocol} {program.extra_params} -e -Ulock:w:0x3F:m -Uefuse:w:0xFD:m -Uhfuse:w:0xDE:m -Ulfuse:w:0xFF:m")
set("ARDUINO_RULE_tools.avrdude.bootloader.pattern" "\"C:/Program Files (x86)/Arduino/hardware/tools/avr/bin/avrdude\" \"-CC:/Program Files (x86)/Arduino/hardware/tools/avr/etc/avrdude.conf\" -v -patmega328p -c{protocol} {program.extra_params} \"-Uflash:w:C:/Program Files (x86)/Arduino/hardware/arduino/avr/bootloaders/optiboot/optiboot_atmega328.hex:i\" -Ulock:w:0x0F:m")
set("ARDUINO_RULE_tools.avrdude_remote.upload.pattern" "/usr/bin/run-avrdude /tmp/sketch.hex {upload.verbose} -patmega328p")



set(CMAKE_C_COMPILER "C:/Program Files (x86)/Arduino/hardware/tools/avr/bin/avr-gcc.exe")
set(CMAKE_C_COMPILE_OBJECT "<CMAKE_C_COMPILER>  -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=108012 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   <DEFINES> <INCLUDES> <FLAGS> <SOURCE> -o <OBJECT>")
set(CMAKE_C_LINK_EXECUTABLE "<CMAKE_C_COMPILER>  -w -Os -g -flto -fuse-linker-plugin -Wl,--gc-sections -mmcu=atmega328p  -o <TARGET> <OBJECTS> <LINK_LIBRARIES>  \"-LD:/Andrew/Documents/Andrew University/Part IV/GDP/GDP-Arduino/Tests/Arduino_Serial_Testing/arduino_serial_test_LED.ino/build\" -lm")
set(CMAKE_C_CREATE_STATIC_LIBRARY "<CMAKE_AR>  rcs  <TARGET> <LINK_FLAGS> <OBJECTS>")

set(CMAKE_CXX_COMPILER "C:/Program Files (x86)/Arduino/hardware/tools/avr/bin/avr-g++.exe")
set(CMAKE_CXX_COMPILE_OBJECT "<CMAKE_CXX_COMPILER>  -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=108012 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   <DEFINES> <INCLUDES> <FLAGS> <SOURCE> -o <OBJECT>")
set(CMAKE_CXX_LINK_EXECUTABLE "<CMAKE_CXX_COMPILER>  -w -Os -g -flto -fuse-linker-plugin -Wl,--gc-sections -mmcu=atmega328p  -o <TARGET> <OBJECTS> <LINK_LIBRARIES>  \"-LD:/Andrew/Documents/Andrew University/Part IV/GDP/GDP-Arduino/Tests/Arduino_Serial_Testing/arduino_serial_test_LED.ino/build\" -lm")
set(CMAKE_CXX_CREATE_STATIC_LIBRARY "<CMAKE_AR>  rcs  <TARGET> <LINK_FLAGS> <OBJECTS>")

set(CMAKE_ASM_COMPILER "C:/Program Files (x86)/Arduino/hardware/tools/avr/bin/avr-gcc.exe")
set(CMAKE_ASM_COMPILE_OBJECT "<CMAKE_ASM_COMPILER>  -c -g -x assembler-with-cpp -flto -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=108012 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   <DEFINES> <INCLUDES> <FLAGS> <SOURCE> -o <OBJECT>")

# Need to include this in cache as plain setting of this variable is
# overwritten when marking it as advanced (This is fixed only in CMake 3.13.0)
set(CMAKE_AR "C:/Program Files (x86)/Arduino/hardware/tools/avr/bin/avr-gcc-ar.exe" CACHE INTERNAL "" FORCE)

set(ARDUINO_FIND_ROOT_PATH "D:/Andrew/Documents/Arduino;C:/Program Files (x86)/Arduino/hardware/arduino/avr;C:/Program Files (x86)/Arduino/hardware/tools/avr;C:/Program Files (x86)/Arduino")
set(ARDUINO_SYSTEM_PROGRAM_PATH "/bin")

