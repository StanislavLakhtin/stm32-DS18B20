cmake_minimum_required(VERSION 3.9)
project(OW CXX C)

IF(NOT CMAKE_CROSSCOMPILING)
    message(FATAL_ERROR "Cross compiling only. Please use -DCMAKE_TOOLCHAIN_FILE=/PATH/TO/TOOLCHAIN_FILE .")
ENDIF(NOT CMAKE_CROSSCOMPILING)

if ((NOT ${CMAKE_C_COMPILER_ID} STREQUAL "GNU") OR
(NOT ${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU") OR
(${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS 7.2.1))
    message(WARNING "This project is targeted for ARM GCC versions 7.2.1 or above. Some features may not be supported\
     by your compiler. You are using:
     ${CMAKE_CXX_COMPILER_ID} C, version ${CMAKE_C_COMPILER_VERSION}
     ${CMAKE_CXX_COMPILER_ID} CXX, version ${CMAKE_CXX_COMPILER_VERSION}")
endif()

# Set the C/C++ standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)

# Choose target device here
set(DEVICE STM32F103C8T6)
set(PLATFORM_ARCH STM32F1)

# Set STM32 SoC specific variables
set(STM32_DEFINES "-DSTM32F1")

# Choose target mainboard pin configuration here
add_definitions(-D${PLATFORM_ARCH}})

# Compiler
include(cmake/toolchain.cmake)

# Compiler
include(cmake/project.cmake)

# Include libopencm3
include(cmake/libopencm3.cmake)

include_directories(
        ${LIBOPENCM3_DIR}/include
        ${PROJECT_SOURCE_DIR}/src)

add_subdirectory(src)

