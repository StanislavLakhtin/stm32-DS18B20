include(CMakeForceCompiler)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR cortex-m3)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)

# Find the cross compiler
find_program(ARM_CC arm-none-eabi-gcc
            ${TOOLCHAIN_DIR}/bin)
find_program(ARM_CXX arm-none-eabi-g++
            ${TOOLCHAIN_DIR}/bin)
find_program(ARM_OBJCOPY arm-none-eabi-objcopy
            ${TOOLCHAIN_DIR}/bin)
find_program(ARM_SIZE_TOOL arm-none-eabi-size
            ${TOOLCHAIN_DIR}/bin)
find_program(ARM_AS arm-none-eabi-as
            ${TOOLCHAIN_DIR}/bin)

CMAKE_FORCE_C_COMPILER(${ARM_CC} GNU)
CMAKE_FORCE_CXX_COMPILER(${ARM_CXX} GNU)

set(CMAKE_ASM_FLAGS
        ${CMAKE_ASM_FLAGS} "-mcpu=cortex-m3 -mthumb")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)