cmake_minimum_required(VERSION 3.13)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/stm32.cmake)

project(stm32-freertos C ASM)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

set(FREERTOS_GIT_URL "github.com/FreeRTOS/FreeRTOS-Kernel")
set(FREERTOS_GIT_TAG V10.4.2)

set(STM32_CMSIS_GIT_TAG v5.6.0)
set(STM32_CMSIS_H7_GIT_TAG v1.10.0)
set(STM32_HAL_H7_GIT_TAG v1.10.0)

find_package(CMSIS COMPONENTS STM32H743ZI_M7 REQUIRED)
find_package(HAL COMPONENTS STM32H743ZI_M7 REQUIRED)
find_package(FreeRTOS COMPONENTS ARM_CM7 REQUIRED)

set(PROJECT_SOURCES
    main.c
    FreeRTOSConfig.h
)

add_executable(stm32-freertos ${PROJECT_SOURCES} stm32h7xx_hal_conf.h)
target_link_libraries(stm32-freertos PRIVATE
    FreeRTOS::Timers
    FreeRTOS::Heap::1
    FreeRTOS::ARM_CM7
    HAL::STM32::H7::M7::RCC
    HAL::STM32::H7::M7::GPIO
    HAL::STM32::H7::M7::CORTEX
    CMSIS::STM32::H743ZI::M7
    STM32::NoSys
)

include(print_target_properties)
print_target_properties(stm32-freertos)
