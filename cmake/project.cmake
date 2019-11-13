# Get Build Information
execute_process(COMMAND make --version OUTPUT_VARIABLE MAKE_OUTPUT)
string(REGEX REPLACE "GNU Make ([0-9]\\.[0-9]\\.*[0-9]*).+" "\\1" MAKE_VERSION ${MAKE_OUTPUT})

message("-----------Build Information------------")
message(STATUS "Host    : ${CMAKE_HOST_SYSTEM_NAME}")
message(STATUS "Make    : ${MAKE_VERSION}")
message(STATUS "CC      : ${CMAKE_CXX_COMPILER_ID} ${CMAKE_C_COMPILER_VERSION}")
message(STATUS "CXX     : ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "Build   : ${CMAKE_BUILD_TYPE}")

set(COMPILER_OPTIONS "-mthumb -mcpu=cortex-m3 -mlittle-endian -mthumb-interwork ")

set(CMAKE_C_FLAGS "${COMPILER_OPTIONS} -lm -lc -lnosys --specs=nosys.specs " )
set(CMAKE_CXX_FLAGS "${COMPILER_OPTIONS} -lm -lc -lnosys --specs=nosys.specs " )
set(CMAKE_ASM_FLAGS "${COMPILER_OPTIONS}")

# Additional Flags
set(ADDITIONAL_C_FLAGS "-fmessage-length=0 -fno-strict-aliasing -ffunction-sections -fdata-sections -fsigned-char")
set(ADDITIONAL_CXX_FLAGS "-fno-exceptions -fno-rtti")
set(ADDITIONAL_LINKER_FLAGS "-Wl,--gc-sections")

# Tune flags
set(ADDITIONAL_C_FLAGS "-mtune=cortex-m3")
set(ADDITIONAL_CXX_FLAGS "-mtune=cortex-m3")

# enable asm for stm startup.s file
enable_language(ASM)

# Enforce C/C++ standard level
set(CMAKE_C_STANDARD_REQUIRED YES)
set(CMAKE_CXX_STANDARD_REQUIRED YES)

# Enforce to disable any compiler-specific extensions
if (ENFORCE_EXTENSION STREQUAL "enforce")
    set(CMAKE_C_EXTENSIONS NO)
    set(CMAKE_CXX_EXTENSIONS NO)
endif()

set(C_SPEC_FLAG "-std=gnu11")
set(CXX_SPEC_FLAG "-std=gnu++14")

set(CMAKE_DEBUG_FLAGS "-g3")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra ${TARGET_FLAGS} ${ADDITIONAL_C_FLAGS} ${C_SPEC_FLAG} ${CMAKE_DEBUG_FLAGS}" CACHE INTERNAL "c compiler flags")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra ${TARGET_FLAGS} ${ADDITIONAL_CXX_FLAGS} ${CXX_SPEC_FLAG} ${CMAKE_DEBUG_FLAGS}" CACHE INTERNAL "cxx compiler flags")
set(CMAKE_ASM_FLAGS "${COMPILER_OPTIONS}" CACHE INTERNAL "asm compiler flags")
set(LINKER_FLAGS "${LINKER_FLAGS} -nostartfiles -lc -lnosys --specs=rdimon.specs ${ADDITIONAL_LINKER_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${LINKER_FLAGS} -T ${CMAKE_SOURCE_DIR}/STM32F103C8T6.ld -lopencm3_stm32f1")

message("------------Additional Flags------------")
message(STATUS "C   : ${ADDITIONAL_C_FLAGS}")
message(STATUS "CXX : ${ADDITIONAL_CXX_FLAGS}")
message(STATUS "LD  : ${ADDITIONAL_LINKER_FLAGS}")
message(STATUS "C SPEC : ${C_SPEC_FLAG}")
message(STATUS "CXX SPEC  : ${CXX_SPEC_FLAG}")

# Build-dependent flags
set(CMAKE_C_FLAGS_DEBUG "-O0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0")
set(CMAKE_C_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(CMAKE_C_FLAGS_MINSIZEREL "-Os")
set(CMAKE_CXX_FLAGS_MINSIZEREL "-Os")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-Og")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-Og")

# Dump all the flags at this point
message(TOUPPER ${CMAKE_BUILD_TYPE} BUILD_TYPE)

message("-------------Full Flag List-------------")
message(STATUS "C   : ${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_${BUILD_TYPE}}")
message(STATUS "CXX : ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${BUILD_TYPE}}")
message(STATUS "LD  : ${CMAKE_EXE_LINKER_FLAGS}")
