cmake_minimum_required(VERSION 3.13)
include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/stm32.cmake)

project(stm32-fetch-cube C ASM)

stm32_fetch_cube(F4 L0)

find_package(CMSIS
             COMPONENTS STM32F407VG STM32L053C8
             REQUIRED)

set(PROJECT_SOURCES main.c)

add_executable(stm32-fetch-f4 ${PROJECT_SOURCES})
target_link_libraries(stm32-fetch-f4 CMSIS::STM32::F407VG STM32::NoSys)

add_executable(stm32-fetch-l0 ${PROJECT_SOURCES})
target_link_libraries(stm32-fetch-l0 CMSIS::STM32::L053C8 STM32::NoSys)
