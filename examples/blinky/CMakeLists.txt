cmake_minimum_required(VERSION 3.13)
include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/stm32.cmake)

project(stm32-blinky C ASM)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

set(STM32_CMSIS_GIT_TAG v5.6.0)
set(STM32_CMSIS_F1_GIT_TAG v4.3.2)
set(STM32_HAL_F1_GIT_TAG v1.1.5)
set(STM32_CMSIS_H7_GIT_TAG v1.10.0)
set(STM32_HAL_H7_GIT_TAG v1.10.0)

find_package(CMSIS COMPONENTS STM32H743ZI_M7 STM32F407VG REQUIRED)
find_package(HAL COMPONENTS STM32H743ZI_M7 STM32F4 RCC GPIO CORTEX REQUIRED)
# Find all drivers:
#find_package(HAL COMPONENTS STM32L0 STM32F1 STM32F4 REQUIRED)
# Find drivers for all families:
#find_package(HAL COMPONENTS RCC GPIO CORTEX REQUIRED)
# Find LL driver:
#find_package(HAL COMPONENTS LL_GPIO REQUIRED)
# Find everything:
#find_package(HAL REQUIRED)

include(${STM32_CMAKE_DIR}/info.cmake)

## STM32F4-Discovery
add_executable(stm32-blinky-f4 blinky.c stm32f4xx_hal_conf.h)
target_link_libraries(stm32-blinky-f4
    HAL::STM32::F4::RCC
    HAL::STM32::F4::GPIO
    HAL::STM32::F4::CORTEX
    CMSIS::STM32::F407VG
    STM32::NoSys
)
#
## STM32VL-Discovery
#add_executable(stm32-blinky-f1 blinky.c stm32f1xx_hal_conf.h)
#target_link_libraries(stm32-blinky-f1
#    HAL::STM32::F1::RCC
#    HAL::STM32::F1::GPIO
#    HAL::STM32::F1::CORTEX
#    CMSIS::STM32::F100RB
#    STM32::NoSys
#)
#
## STM32L0538-Discovery
#add_executable(stm32-blinky-l0 blinky.c stm32l0xx_hal_conf.h)
#target_link_libraries(stm32-blinky-l0
#    HAL::STM32::L0::RCC
#    HAL::STM32::L0::GPIO
#    HAL::STM32::L0::CORTEX
#    CMSIS::STM32::L053C8
#    STM32::NoSys
#)

# NUCLEO-H743ZI
add_executable(stm32-blinky-h7 blinky.c stm32h7xx_hal_conf.h)
target_link_libraries(stm32-blinky-h7
    HAL::STM32::H7::M7::RCC
    HAL::STM32::H7::M7::GPIO
    HAL::STM32::H7::M7::CORTEX
    CMSIS::STM32::H743ZI::M7
    STM32::NoSys
)