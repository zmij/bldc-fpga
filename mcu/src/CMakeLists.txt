cmake_minimum_required(VERSION 3.25)

file(
    GLOB_RECURSE 
    SDK_FILES
    LIST_DIRECTORIES FALSE
    ../lib/*.c
)

set(
    ${PROJECT_NAME}_SRC 
    # ${SDK_FILES}
    startup_gw1ns4c.s
    main.cpp    
)

add_firmware(
    ${PROJECT_NAME}
    SOURCES ${${PROJECT_NAME}_SRC}
    INCLUDE_DIRS
        ${PROJECT_SOURCE_DIR}/lib/CMSIS/CoreSupport/gmd
        ${PROJECT_SOURCE_DIR}/lib/CMSIS/DeviceSupport/system
        ${PROJECT_SOURCE_DIR}/lib/StdPeriph_Driver/Includes
        ${PROJECT_SOURCE_DIR}/template
    LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/link.ld
    LINK_TARGETS armpp
)
