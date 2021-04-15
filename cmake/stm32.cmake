set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/stm32_gcc.cmake)

function(get_openocd_config_name_for_target TARGET CFG)
    get_target_property(LIBRARIES ${TARGET} LINK_LIBRARIES)
    list(FILTER LIBRARIES INCLUDE REGEX "CMSIS::STM32")
    #foreach(LIBRARY ${LIBRARIES})
    #    message(STATUS ${LIBRARY})
    #endforeach()
    list(LENGTH LIBRARIES NUM_LIBRARIES)
    if(NUM_LIBRARIES EQUAL 1)
        get_target_property(DEPENDENCIES ${LIBRARIES} MANUALLY_ADDED_DEPENDENCIES)
        list(FILTER DEPENDENCIES INCLUDE REGEX "OPENOCD")
        list(LENGTH DEPENDENCIES NUM_DEPENDENCIES)
        if(NUM_DEPENDENCIES EQUAL 1)
            message("OpenOCD config file defined: ${DEPENDENCIES}.cfg")
            set(${CFG} ${DEPENDENCIES}.cfg PARENT_SCOPE)
        endif()
    endif()
endfunction()

function(stm32_add_flash_target TARGET)
    if(TARGET flash_${TARGET})
        return()
    endif()

    get_property(TMP TARGET ${TARGET} PROPERTY TYPE)
    if(NOT TMP STREQUAL "EXECUTABLE")
        return()
    endif()

    # Find OpenOCD path
    find_program(OPENOCD_BIN "openocd")
    if(OPENOCD_BIN)
        get_filename_component(OPENOCD_PATH ${OPENOCD_BIN} DIRECTORY)
        get_filename_component(OPENOCD_PATH ${OPENOCD_PATH} DIRECTORY)
        #message("OpenOCD path: ${OPENOCD_PATH}")

        get_openocd_config_name_for_target(${TARGET} OPENOCD_CFG_NAME)
        if(NOT OPENOCD_CFG_NAME)
            return()
        endif()

        set(OPENOCD_CFG "${CMAKE_CURRENT_BINARY_DIR}/${OPENOCD_CFG_NAME}")
        #message("OpenOCD config file: ${OPENOCD_CFG}")
#        add_custom_command(OUTPUT "${OPENOCD_CFG}"
#                COMMAND ${CMAKE_COMMAND}
#                -DOPENOCD_CFG="${OPENOCD_CFG}"
#                -P "${STM32_CMAKE_DIR}/stm32/openocd_cfg.cmake"
#                )

        add_custom_target(flash_${TARGET}
                COMMAND ${OPENOCD_BIN} -f ${OPENOCD_CFG} -c "gdb_memory_map disable"
                -c "init"
                -c "targets"
                -c "reset halt"
                #-c "load_image $<TARGET_FILE:${PROJECT_NAME}.elf>"
                -c "flash write_image erase $<TARGET_FILE:${TARGET}>"
                #-c "flash write_image erase ${BIN_FILE} 0x08000000"
                -c "reset halt"
                -c "verify_image $<TARGET_FILE:${TARGET}>"
                -c "reset run"
                -c "exit" # exit, resume or shutdown
                WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                DEPENDS ${OPENOCD_CFG} $<TARGET_FILE:${TARGET}>
                )

        add_custom_target(openocd_${TARGET}
                COMMAND ${OPENOCD_BIN} -f ${OPENOCD_CFG} -c "gdb_memory_map disable"
                WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                DEPENDS ${OPENOCD_CFG} $<TARGET_FILE:${TARGET}>
                )        
    endif()
endfunction()

function(stm32_add_gdb_target TARGET)
    if(TARGET gdb_${TARGET})
        return()
    endif()

    # Start debugging with GDB (openocd needs to run in background)
    add_custom_target(gdb_${TARGET}
        COMMAND ${CMAKE_GDB} -x ${STM32_CMAKE_DIR}/gdb.cfg $<TARGET_FILE:${TARGET}>
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        DEPENDS ${OPENOCD_CFG} $<TARGET_FILE:${TARGET}>
    )    
endfunction()

function(stm32_convert_to_hex TARGET)
    #     add_custom_command(TARGET ${TARGET}
    #             POST_BUILD
    #             COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${TARGET}> $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.hex
    #             COMMENT "Generating HEX file for ${TARGET}")
    add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${TARGET}.hex"
            COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${TARGET}> $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.hex
            COMMENT "Generating HEX file for ${TARGET}")
    add_custom_target(${TARGET}_hex
            COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${TARGET}> $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.hex
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            DEPENDS $<TARGET_FILE:${TARGET}>
            COMMENT "Generating HEX file for ${TARGET}"
            )
    set_property(
            TARGET ${TARGET}
            APPEND
            PROPERTY ADDITIONAL_CLEAN_FILES $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.hex # add the hex file to be removed when running 'make clean'
    )
endfunction()

function(stm32_convert_to_binary TARGET)
    #    add_custom_command(TARGET ${TARGET}
    #            POST_BUILD
    #            COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET}> $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.bin
    #            COMMENT "Generating BIN file for ${TARGET}")
    add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${TARGET}.bin"
            COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET}> $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.bin
            COMMENT "Generating HEX file for ${TARGET}")
    add_custom_target(${TARGET}_bin
            COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET}> $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.bin
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            DEPENDS $<TARGET_FILE:${TARGET}>
            COMMENT "Generating BIN file for ${TARGET}"
            )
    set_property(
            TARGET ${TARGET}
            APPEND
            PROPERTY ADDITIONAL_CLEAN_FILES $<TARGET_FILE_DIR:${TARGET}>/${TARGET}.bin # add the binary file to be removed when running 'make clean'
    )
endfunction()

function(stm32_print_target_size_after_build TARGET)
    add_custom_command(TARGET ${TARGET}
            POST_BUILD
            COMMAND echo "\n"
            COMMAND ${CMAKE_SIZE} "$<TARGET_FILE:${TARGET}>")
endfunction()

# Tap into (override) add_executable to add a few extra functions
function(add_executable TARGET)
    _add_executable(${TARGET} ${ARGN}) # Call the original function
    stm32_print_target_size_after_build(${TARGET})
    #stm32_add_flash_target(${TARGET})
    stm32_convert_to_hex(${TARGET})
    stm32_convert_to_binary(${TARGET})
    stm32_add_gdb_target(${TARGET})
endfunction()

function(target_link_libraries TARGET)
    _target_link_libraries(${TARGET} ${ARGN}) # Call the original function
    stm32_add_flash_target(${TARGET})
endfunction()