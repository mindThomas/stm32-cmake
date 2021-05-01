cmake_minimum_required(VERSION 3.13)
include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/stm32.cmake)

project(stm32-custom-linker-script C ASM)

find_package(CMSIS
             COMPONENTS STM32F407VG STM32H743ZI_M7
             REQUIRED)

set(PROJECT_SOURCES main.c)

add_executable(stm32-custom-linker-script-f4 ${PROJECT_SOURCES})
target_link_libraries(stm32-custom-linker-script-f4 CMSIS::STM32::F407xx STM32::NoSys)
stm32_add_linker_script(stm32-custom-linker-script-f4 PRIVATE F407VG.ld)

add_executable(stm32-custom-linker-script-h7 ${PROJECT_SOURCES})
target_link_libraries(stm32-custom-linker-script-h7 CMSIS::STM32::H743xx::M7 STM32::NoSys)
stm32_add_linker_script(stm32-custom-linker-script-h7 PRIVATE H743ZIT.ld)
