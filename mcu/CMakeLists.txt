cmake_minimum_required(VERSION 3.25)

#set(CMAKE_VERBOSE_MAKEFILE TRUE)

set(TARGET_PROCESSOR "cortex-m3" CACHE STRING "Target processor")

include(lib/armpp/cmake/arm-none-eabi-gcc.cmake)
include(lib/armpp/cmake/firmware_funcs.cmake)

project(
    bldc-fpga 
    VERSION 0.1
    DESCRIPTION "Firmware for Gowin GW1NSR ARM EMPU for driving a bldc motor"
    LANGUAGES C CXX ASM
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)

add_subdirectory(lib/armpp)
add_subdirectory(src)
