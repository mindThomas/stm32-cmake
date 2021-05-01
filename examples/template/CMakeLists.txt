cmake_minimum_required(VERSION 3.13)
include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/stm32.cmake)

project(stm32-template C ASM)

find_package(CMSIS
             COMPONENTS STM32F407VG
             REQUIRED)
# Find CMSIS for all F4 devices:
#find_package(CMSIS COMPONENTS STM32F4 REQUIRED)
# Find CMSIS for all devices:
#find_package(CMSIS REQUIRED)

set(PROJECT_SOURCES main.c)

add_executable(stm32-template ${PROJECT_SOURCES})
target_link_libraries(stm32-template CMSIS::STM32::F407VG STM32::NoSys)
