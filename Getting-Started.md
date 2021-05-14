# Getting Started

## Create project in CubeMX
It is recommended that you create a project with CubeMX to generate processor configuration files including:

 - `stm32xyzz_hal_msp.c`
 - `stm32xyzz_it.c`
 - `syscalls.c`

## ARM Toolchain
This project requires [CMake](https://cmake.org/download/) 3.15 or later and the [arm-none-eabi-gcc toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads) to be installed seperately.
This project was recently tested and used with the [GNU Arm Embedded Toolchain Version 7-2018-q2-update](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads/7-2018-q2-update).
OpenOCD is used for flashing and debugging. The project has been tested with [xpack-openocd-0.11.0](https://xpack.github.io/blog/2021/03/15/openocd-v0-11-0-1-released/) on Ubuntu 18.04.

Add/set the path to your ARM toolchain folder (which contains the executable `arm-none-eabi-gcc` and more) in your `.bashrc`:
```
export STM32_TOOLCHAIN_PATH=/path/to/gcc-arm-none-eabi-xxx
export ARM_TOOLCHAIN_DIR=$STM32_TOOLCHAIN_PATH/bin
export PATH=$PATH:/path/to/xpack-openocd-0.11.0-1/bin
```

## IDE
Please note that [CLion](https://www.jetbrains.com/clion/) can be used to load the project and comes bundled with OpenOCD: https://www.jetbrains.com/help/clion/openocd-support.html
Just select the automatically generated OpenOCD configuration file when creating the __OpenOCD Download & Run__ configuration. Remember also to set:
 - `Download` --> `Updated Only`
 - `Reset` --> `Halt`   _(this appears to be highly important, otherwise registers etc. is not reset/cleared after starting a new debug and flashing)_

Alternatively VS Code can be used: https://medium.com/@lixis630/getting-started-to-code-embedded-c-on-stm32-e90e7910b2c

## Create a new project
```bash
wget -O /tmp/stm32-cmake.zip https://github.com/mindThomas/stm32-cmake/archive/refs/heads/master.zip
unzip /tmp/stm32-cmake.zip -d /tmp/stm32-cmake
wget -O /tmp/STM32-libraries.zip https://github.com/mindThomas/STM32-libraries/archive/8473d034e14dbd1f1bb21520bb70072862bc867a.zip
unzip /tmp/STM32-libraries.zip -d /tmp/STM32-libraries
to_delete="/tmp/stm32-cmake.zip:/tmp/STM32-libraries.zip:/tmp/stm32-cmake:/tmp/STM32-libraries"

mkdir ~/my_project
cd ~/my_project
mv /tmp/stm32-cmake/**/.clang-format .
mv /tmp/stm32-cmake/**/clang-format.sh .
mv /tmp/stm32-cmake/**/.cmake-format .
mv /tmp/stm32-cmake/**/cmake-format.sh .
mv /tmp/stm32-cmake/**/cmake .
mkdir STM32-libraries
mv /tmp/STM32-libraries/**/* ./STM32-libraries
cp STM32-libraries/Templates/Priorities.h .
cp STM32-libraries/Templates/FreeRTOSConfig.h .
touch CMakeLists.txt

for path in ${to_delete//:/ }; do rm -rf $path; done
```

## CMakeLists content
Example with STM32G431CB device:
```
cmake_minimum_required(VERSION 3.15)

include(STM32-libraries/versions.cmake) # Defines HAL, CMSIS and FreeRTOS versions (Git tags) to use
include(cmake/stm32.cmake)

project(MyProject C CXX ASM)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

find_package(CMSIS
             COMPONENTS STM32G431CB    # specify your exact processor
             REQUIRED)
find_package(HAL
             COMPONENTS STM32G431CB    # specify your exact processor
             REQUIRED)
find_package(FreeRTOS
             COMPONENTS ARM_CM4F
             REQUIRED)

# Add STM32-libraries  (has to come after find_package)
add_subdirectory(STM32-libraries)

# Add the sources to the firmware
add_executable(firmware main.c)

# Set external (HSE) oscillator frequency
target_compile_definitions(firmware PUBLIC HSE_VALUE=8000000) # specify HSE crystal as 8 MHz

# Add libraries to link to the firmware
target_link_libraries(firmware PUBLIC
                      # Processor dependencies
                      CMSIS::STM32::G431CB # generates and adds the linker file and startup script
                      STM32::Nano # NoSys or Nano specs
                      Syscalls  # default implementation of syscalls (not needed if NoSys is used)
                      # HAL libraries for CPU configuration (defines USE_HAL_DRIVER)
                      HAL::STM32::G4::RCC
                      HAL::STM32::G4::RCCEx
                      HAL::STM32::G4::PWR
                      HAL::STM32::G4::PWREx
                      HAL::STM32::G4::CORTEX
                      DefaultInterrupts  # Default interrupts including default SysTick
                      # Periphiral libraries (e.g. from STM32-libraries)
                      STM32G4_IO)
                      
# Include default SysTick interrupt increasing HAL tick
enable_option(firmware DefaultInterrupts Systick)
```

### Custom Linker file or Startup script
With the CMake above where the exact processor is specified, the Linker file and Startup script is automatically generated.
If you want to specify these manually you can use:
```
stm32_add_linker_script(firmware  relpath/to/linker.ld)
stm32_add_startup_script(firmware relpath/to/startup_stm32xyyyzz.s)
```

### Enable C++ usage = syscalls (new, malloc etc.)
To use malloc, free, new and other system libraries an implementation of the syscalls functions and the memory block allocation function `_sbrk` has to be given. A default implementation can be used by linking against:
```
Syscalls
```
Alternatively the bigger suite of C/C++ libraries can be included with:
```
STM32::NoSys
```

### Enable printf
To use printf you will have to enable `NoSys` or `Nano` specs.
You get the full printf suite with `NoSys` by linking against:
```
STM32::NoSys
```
This will however also require the largest amount of FLASH and RAM. To use a smaller (`Nano`) version you should link against:
```
STM32::Nano
```
To use the printf libraries with the `Nano` version you will also need to define the Syscalls functions mentioned above. A default implementation can be included by linking against:
```
Syscalls
```

#### Enable floating point printf with Nano specs
Out of the box `Nano` specs do not support floating point values with printf. To enable this you must add:
```
enable_option(firmware STM32::Nano FLOAT)
```
Note that this will take up approximately 8 KB of FLASH.

### Use CMSIS-RTOS with FreeRTOS
The CMSIS RTOS definitions are different from FreeRTOS but a compatibility layer can be added by linking to:
```
FreeRTOS::CMSIS
```

### Enabling library options
Within `STM32-libraries` each library might have specific options that can be enabled. This is done with:
```
enable_option(TARGET LIBRARY OPTION)
```
For example:
```
enable_option(firmware STM32G4_IO DEBUG)
enable_option(firmware LSPC  UART)
enable_option(firmware Debug LSPC)
enable_option(firmware Debug PRINTF)
enable_option(firmware CPULoad VERBOSE)
```

When enabling an option the following compiler symbol is defined:
```
<LIBRARY>_USE_<OPTION>
```

### Setting configurable library options
Similar to library options that can be enabled some of the libraries in `STM32-libraries` also have some configurable options which are set with:
```
set_configurable_option(TARGET LIBRARY OPTION VALUE)
```
For example:
```
set_configurable_option(firmware STM32H7_PRECISION_SYSTICK FREQUENCY 10000)
set_configurable_option(firmware STM32H7_USBCDC DEVICE_NAME "My STM32")
```

When setting a configurable option the following compiler symbol is defined:
```
<LIBRARY>_<OPTION>=<VALUE>
```

### Using the fetch feature alternative
__OBS. This is experimental and should not be used__
The HAL Library, CMSIS, FreeRTOS etc. will be downloaded automatically when using this CMake project. 
Another fetch method is also supported which however will fetch the whole repository of components instead of just the needed tag/version specified in the `versions.cmake` file.

Example to download CMSIS and HAL for STM32G4:
```
stm32_fetch_cmsis(G4)
stm32_fetch_hal(G4)
```

## Main code - LED blink example
Example of `main.cpp` for project with STM32G431CB device to blink an LED:
```
#include "stm32g4xx_hal.h"

#include <IO/IO.hpp>

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 42;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_8) != HAL_OK)
  {
    Error_Handler();
  }
}

int main(void)
{
    // Reset of all peripherals, Initializes the Flash interface and the Systick.
    HAL_Init();
  
    // Configure the system clock
    SystemClock_Config();

    // Create test LED
    IO led(GPIOA, GPIO_PIN_5);

    // Infinite loop
    while (1)
    {
        led.Toggle();
        HAL_Delay(500);
    }
}
```

## FreeRTOS Example
A working example with FreeRTOS can quickly be achieved by slight modification to the previous example.
Modify the `CMakeLists.txt` file by removing the `Systick` option from the `DefaultInterrupts` and add the following libraries to the linking list:
```
FreeRTOS::ARM_CM4F # defines USE_FREERTOS
FreeRTOS::Timers
FreeRTOS::Heap::4 # use the Heap 4 type with FreeRTOS (generally recommended)
```

Now replace the main function with the following:
```
void testTask(void *arg)
{
    IO led(GPIOA, GPIO_PIN_5);

    while (true)
    {
        led.Toggle();
        vTaskDelay(500);
    }
}

int main(void)
{
    // Reset of all peripherals, Initializes the Flash interface and the Systick.
    HAL_Init();
  
    // Configure the system clock
    SystemClock_Config();

    // Create test task which will blink the LED
    xTaskCreate(testTask, "testTask", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();
    for (;;);

    return 0;
}
```

As a final step you need to add the FreeRTOS include file in the top:
```
#include <FreeRTOS.h>
```

## DFU Flashing
A firmware can be flashed over UART or USB using the built in DFU bootloader of ST devices. For USB enabled devices `dfu-util` tool from http://dfu-util.sourceforge.net/dfuse.html can be used.

### List connected devices

Run the following command to list the connected DFU devices:

`dfu-util -l`

### Flashing the firmware

To flash a firmware use the following command:

`dfu-util -a 0 -i 0 -s 0x08000000:leave -D firmware.bin`

### Getting out of DFU

Ideally one should be able to use the command:

`dfu-util -a 0 -e -R`

But it is not always that it works. Alternatively the `dfu-tool` can be used by running:

`dfu-tool attach`

If none of the above commands work (experienced with the STM32F4 Blackpill board) a last resort is to trick the STM32 into resetting by flashing 0 bytes:

```
touch null.bin
dfu-util -a 0 -i 0 -s 0x08000001:leave -D null.bin
```





## Debugging in CLion
Debugging with an ST-Link can be done with CLion in 3 different ways:
1. OpenOCD using bundled OpenOCD (simplest): https://www.jetbrains.com/help/clion/openocd-support.html
2. Using open source `st-util` GDB server: https://github.com/stlink-org/stlink
3. Using ST-LINK GDB server: https://nicolai86.eu/blog/2020/08/on-chip-debugging-stm32/

### Bundled OpenOCD
This is by far the easiest option as it just requires you to create an `OpenOCD Download & Run` debug configuration in CLion and configure it as follows:
1. Set the target and executable to your firmware
2. Set the GDB to `Bundled GDB`
3. Set the Download executable to `Always`.
4. Select the auto-generate OpenOCD Board config file
5. Set Download to `Updated Only`
6. Set Reset to `Halt`

### Open Source `st-util` GDB server
Alternatively the open source `st-util` can be used as GDB server. To do so you should create an `Embedded GDB Server` debug configuration in CLion instead.

Clone the `stlink` repo from and build it by doing:
```
git clone https://github.com/stlink-org/stlink
cd stlink
git checkout develop
make clean
make release
```
Now you will have the `st-util` in `stlink/build/Release/bin`.

In the CLion debug config do the following:
1. Set the GDB to `Bundled GDB`
2. Set the Download executable to `Always`.
3. Set the GDB Server to the `st-util` binary
4. Set the `target remote` args to: `localhost:4242`

Hint: If you are struggling with breakpoints not getting hit make sure that you don't have too many (max 6 breakpoints) and make sure that your firmware was flashed. If your debug configuration is set to `Download executable: Updated Only` and the upload fails, CLion might think that the firmware has been uploaded and whenever you start a new debug session it won't reupload. So a general advice is to set the Download executable to `Always`.

### ST-LINK GDB server
This requires an existing installation of STM32CubeIDE.
Find the installation folder and find the `stlink-gdb-server` subfolder under plugins. Note this as `GDB_SERVER_PATH`.
Next find the `cubeprogrammer` subfolder also under plugins. Note this as `CUBE_PROGRAMMER_PATH`.
Create a GDB server launch script as `stlink_gdb_launch.sh`:
```
#!/bin/sh
GDB_SERVER_PATH=~/st/stm32cubeide_1.4.0/plugins/com.st.stm32cube.ide.mcu.externaltools.stlink-gdb-server.linux64_1.6.0.202101291314/tools/bin
CUBE_PROGRAMMER_PATH=~/st/stm32cubeide_1.4.0/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.linux64_1.4.0.202007081208/tools/bin

PATH=$PATH:$GDB_SERVER_PATH
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$GDB_SERVER_PATH/native/linux_x64/

ST-LINK_gdbserver -e -f debug.log -p 61234 -r 15 -d -cp $CUBE_PROGRAMMER_PATH
```
Finally create an `Embedded GDB Server` configuration in CLion where you:
1. Use `Bundled GDB`
2. Set the Download executable to `Always`.
3. Set `target remote` args to: `localhos:61234`
4. Set the GDB Server to the `stlink_gdb_launch.sh` script