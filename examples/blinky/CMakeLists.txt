cmake_minimum_required(VERSION 3.13)
include(${CMAKE_CURRENT_SOURCE_DIR}/../../STM32-libraries/versions.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/stm32.cmake)

project(stm32-blinky C CXX ASM)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

# The Fetch feature can also be used, but that will fetch the whole repository of the components
# instead of just the needed tag
#stm32_fetch_cmsis(H7)
#stm32_fetch_hal(H7)

find_package(CMSIS
             COMPONENTS STM32H743ZI_M7 STM32F407VG
             REQUIRED)
#find_package(HAL COMPONENTS STM32H7_M7 STM32F4 RCC GPIO FLASH CORTEX REQUIRED)
find_package(HAL
             COMPONENTS STM32H7_M7 STM32F4
             REQUIRED)
find_package(FreeRTOS
             COMPONENTS ARM_CM7
             REQUIRED)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../STM32-libraries stm32-libraries)
# has to come after find_package
# Find all drivers:
#find_package(HAL COMPONENTS STM32L0 STM32F1 STM32F4 REQUIRED)
# Find drivers for all families:
#find_package(HAL COMPONENTS RCC GPIO CORTEX REQUIRED)
# Find LL driver:
#find_package(HAL COMPONENTS LL_GPIO REQUIRED)
# Find everything:
#find_package(HAL REQUIRED)

include(${STM32_CMAKE_DIR}/info.cmake)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# STM32F4-Discovery
#add_executable(stm32-blinky-f4 blinky.cpp stm32f4xx_hal_conf.h)
#target_link_libraries(stm32-blinky-f4
#    HAL::STM32::F4::RCC
#    HAL::STM32::F4::GPIO
#    HAL::STM32::F4::CORTEX
#    CMSIS::STM32::F407VG
#    STM32::NoSys
#    STM32F4_IO
#)
#
# STM32VL-Discovery
#add_executable(stm32-blinky-f1 blinky.c stm32f1xx_hal_conf.h)
#target_link_libraries(stm32-blinky-f1
#    HAL::STM32::F1::RCC
#    HAL::STM32::F1::GPIO
#    HAL::STM32::F1::CORTEX
#    CMSIS::STM32::F100RB
#    STM32::NoSys
#)
#
# STM32L0538-Discovery
#add_executable(stm32-blinky-l0 blinky.c stm32l0xx_hal_conf.h)
#target_link_libraries(stm32-blinky-l0
#    HAL::STM32::L0::RCC
#    HAL::STM32::L0::GPIO
#    HAL::STM32::L0::CORTEX
#    CMSIS::STM32::L053C8
#    STM32::NoSys
#)

# NUCLEO-H743ZI
add_executable(stm32-blinky-h7
               blinky.cpp
               syscalls.c
               sysmem.c)
# Try without these libraries. The <string.h> library (needed for memset) is apparently requiring abort, _exit etc. to be defined
target_link_libraries(stm32-blinky-h7
                      HAL::STM32::H7::M7::RCC
                      HAL::STM32::H7::M7::RCCEx
                      HAL::STM32::H7::M7::PWR
                      #HAL::STM32::H7::M7::FLASH
                      HAL::STM32::H7::M7::GPIO
                      HAL::STM32::H7::M7::CORTEX
                      CMSIS::STM32::H743ZI::M7
                      STM32::Nano # NoSys or Nano specs
                      STM32H7_IO
                      #STM32H7_ADC
                      #STM32H7_EEPROM
                      #STM32H7_ENCODER
                      #STM32H7_I2C
                      #STM32H7_INPUTCAPTURE
                      STM32H7_PWM
                      STM32H7_SMBUS
                      STM32H7_SPI
                      STM32H7_TIMER
                      STM32H7_UART
                      STM32H7_USBCDC)
target_compile_definitions(stm32-blinky-h7 PUBLIC HSE_VALUE=8000000)

#target_compile_definitions(stm32-blinky-h7 PUBLIC ARM_CPU=)

#target_compile_definitions(
#        stm32-blinky-h7 PUBLIC
#        $<FILTER:$<TARGET_PROPERTY:COMPILE_OPTIONS>,INCLUDE,mcpu>)

#add_custom_target(genexdebug COMMAND ${CMAKE_COMMAND} -E echo "$<FILTER:$<TARGET_PROPERTY:stm32-blinky-h7,COMPILE_OPTIONS>,INCLUDE,mcpu>")
#get_target_property(options stm32-blinky-h7 LINK_OPTIONS)
#message("Options = ${options}")

include(print_target_properties)
#get_propagated_target_properties(outlist stm32-blinky-h7 INTERFACE_COMPILE_OPTIONS)
#message("${outlist}")

get_mcpu(mcpu stm32-blinky-h7)
message("MCPU=${mcpu}")

get_property(name TARGET stm32-blinky-h7 PROPERTY DEVICE_NAME)
get_property(family TARGET stm32-blinky-h7 PROPERTY DEVICE_FAMILY_SHORT)
message("Name = ${name} | Family = ${family}")

#enable_option(stm32-blinky-h7 STM32H7_IO "DEBUG") # defines the symbol: "STM32H7_IO_USE_DEBUG"
#set_configurable_option(stm32-blinky-h7 STM32H7_USBCDC DEVICE_NAME "My STM32")
