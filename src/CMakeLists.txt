cmake_minimum_required(VERSION 3.9)

file(GLOB_RECURSE SOURCES *.cpp *.c )
file(GLOB_RECURSE C_SOURCES *.c )
set_source_files_properties(C_SOURCES PROPRTIES LANGUAGE C)

include_directories(./include)

message(STATUS "SOURCE FILES : ${SOURCES}")

add_executable(${PROJECT_NAME}.elf ${SOURCES})

target_link_libraries(${PROJECT_NAME}.elf opencm3_stm32f1)

