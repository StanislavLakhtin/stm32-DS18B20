cmake_minimum_required(VERSION 3.5.0)
project(stm32-DS18x20)

IF(NOT CMAKE_CROSSCOMPILING)
    message(FATAL_ERROR "Cross compiling only. Please use -DCMAKE_TOOLCHAIN_FILE=/PATH/TO/TOOLCHAIN_FILE .")
ENDIF(NOT CMAKE_CROSSCOMPILING)

add_definitions(-DSTM32F1)

set(LIBOPENCM3_DIR ${CMAKE_SOURCE_DIR}/libopencm3)
add_custom_target(libopencm3 make WORKING_DIRECTORY ${LIBOPENCM3_DIR})
link_directories(${LIBOPENCM3_DIR}/lib)

#file(GLOB_RECURSE USER_SOURCES src/*.c)

include_directories(
        ${LIBOPENCM3_DIR}/include
        inc)

#CFLAGS="-fshort-wchar" make    # compile lib with 2 byte wide wchar_t

set(STM32F1_FLAGS "-mcpu=cortex-m3 -mthumb -mthumb-interwork -msoft-float")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall ${STM32F1_FLAGS} ")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -Wall --std=c++14 ${STM32F1_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T ${CMAKE_SOURCE_DIR}/libopencm3.ld -nostartfiles --specs=rdimon.specs -Wl,-Map=${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.map")


function(add_bin_from_elf bin elf)
    add_custom_target(${bin}
            ALL ${CMAKE_OBJCOPY}
            -Obinary ${elf} ${bin} DEPENDS ${elf})
endfunction(add_bin_from_elf)

add_subdirectory(src)