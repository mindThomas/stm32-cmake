cmake_minimum_required(VERSION 3.13)
include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/stm32.cmake)

project(stm32-freertos C CXX ASM)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

set(STM32_CMSIS_GIT_TAG v5.6.0)
set(STM32_CMSIS_H7_GIT_TAG v1.10.0)
set(STM32_HAL_H7_GIT_TAG v1.10.0)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../STM32-libraries/versions.cmake)
find_package(CMSIS
             COMPONENTS STM32H743ZI_M7
             REQUIRED)
find_package(HAL
             COMPONENTS STM32H743ZI_M7
             REQUIRED)
find_package(FreeRTOS
             COMPONENTS ARM_CM7
             REQUIRED)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../STM32-libraries stm32-libraries) # has to come after find_package

set(PROJECT_SOURCES main.cpp FreeRTOSConfig.h)

add_executable(stm32-freertos ${PROJECT_SOURCES} stm32h7xx_hal_conf.h)
target_link_libraries(stm32-freertos
                      PRIVATE FreeRTOS::Timers
                              FreeRTOS::Heap::1
                              FreeRTOS::ARM_CM7
                              HAL::STM32::H7::M7::RCC
                              HAL::STM32::H7::M7::RCCEx
                              HAL::STM32::H7::M7::PWR
                              HAL::STM32::H7::M7::PWREx
                              HAL::STM32::H7::M7::GPIO
                              HAL::STM32::H7::M7::CORTEX
                              CMSIS::STM32::H743ZI::M7
                              STM32::NoSys
        STM32H7_PrecisionSysTick
                              STM32H7_IO
                              STM32H7_USBCDC)
target_compile_definitions(stm32-freertos PUBLIC HSE_VALUE=8000000)

#include(print_target_properties)
#print_target_properties(stm32-freertos)

enable_option(stm32-freertos STM32H7_IO "DEBUG")
# defines the symbol: "STM32H7_IO_USE_DEBUG"
set_configurable_option(stm32-freertos STM32H7_PRECISION_SYSTICK FREQUENCY 10000)
set_configurable_option(stm32-freertos STM32H7_USBCDC DEVICE_NAME "My STM32")
