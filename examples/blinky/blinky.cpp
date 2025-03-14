#if defined STM32L0
    #include <stm32l0xx_hal.h>

    // STM32L0538-Discovery green led - PB4
    #define LED_PORT                GPIOB
    #define LED_PIN                 GPIO_PIN_4
    #define LED_PORT_CLK_ENABLE     __HAL_RCC_GPIOB_CLK_ENABLE
#elif defined STM32F1
    #include <stm32f1xx_hal.h>

    // STM32VL-Discovery green led - PC9
    #define LED_PORT                GPIOC
    #define LED_PIN                 GPIO_PIN_9
    #define LED_PORT_CLK_ENABLE     __HAL_RCC_GPIOC_CLK_ENABLE
#elif defined STM32F4
    #include <stm32f4xx_hal.h>

    // STM32F4-Discovery green led - PD12
    #define LED_PORT                GPIOD
    #define LED_PIN                 GPIO_PIN_12
    #define LED_PORT_CLK_ENABLE     __HAL_RCC_GPIOD_CLK_ENABLE
#elif defined STM32H7
    #include <stm32h7xx_hal.h>

    // STM32F4-Discovery green led - PD12
    #define LED_PORT                GPIOB
    #define LED_PIN                 GPIO_PIN_0
    #define LED_PORT_CLK_ENABLE     __HAL_RCC_GPIOB_CLK_ENABLE
#endif

#include <IO/IO.hpp>
IO led(LED_PORT, LED_PIN);

//#include <PWM/PWM.hpp>
//PWM pwm(PWM::TIMER3, PWM::CH3, 100, 100);

#include <Timer/Timer.hpp>
Timer timer(Timer::TIMER7, 1000);

#include <math.h>
#include <cmath>
#include <cstdlib>

#include <USBCDC/USBCDC.hpp>

uint16_t value = 0;

// OBS. Interrupts has to be defined as C functions (avoid name mingling to retain the correct name)
#ifdef __cplusplus
extern "C" {
#endif

void SysTick_Handler(void)
{
    HAL_IncTick();

    // 1 Hz blinking
    if ((HAL_GetTick() % 100) == 0) {
        value += 10;
        if (value > 100) {
            value = 0;
        }
        //pwm.SetRaw(std::abs((int16_t)value - 50));
    }
        //HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        //led.Toggle();
}

#ifdef __cplusplus
}
#endif

void blink()
{
    led.Toggle();
}

void initGPIO()
{
    GPIO_InitTypeDef GPIO_Config;

    GPIO_Config.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Config.Pull = GPIO_NOPULL;
    GPIO_Config.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_Config.Pin = LED_PIN;

    LED_PORT_CLK_ENABLE();
    HAL_GPIO_Init(LED_PORT, &GPIO_Config);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    /** Supply configuration update enable
    */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    /** Configure the main internal regulator output voltage
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
    RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 400;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLQ = 4;

    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        while(1);
    }
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                                  |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        while(1);
    }

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_UART4
                                               |RCC_PERIPHCLK_UART7|RCC_PERIPHCLK_SPI3
                                               |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_I2C3
                                               |RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_I2C1
                                               |RCC_PERIPHCLK_SPI6|RCC_PERIPHCLK_I2C4
                                               |RCC_PERIPHCLK_USB;
    PeriphClkInitStruct.PLL2.PLL2M = 1;
    PeriphClkInitStruct.PLL2.PLL2N = 50;
    PeriphClkInitStruct.PLL2.PLL2P = 5;
    PeriphClkInitStruct.PLL2.PLL2Q = 5;
    PeriphClkInitStruct.PLL2.PLL2R = 2;
    PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
    PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
    PeriphClkInitStruct.PLL3.PLL3M = 1;
    PeriphClkInitStruct.PLL3.PLL3N = 42;
    PeriphClkInitStruct.PLL3.PLL3P = 2;
    PeriphClkInitStruct.PLL3.PLL3Q = 7;
    PeriphClkInitStruct.PLL3.PLL3R = 2;
    PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_3;
    PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
    PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
    PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
    PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_D3PCLK1;
    PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
    PeriphClkInitStruct.Spi6ClockSelection = RCC_SPI6CLKSOURCE_PLL2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        while(1);
    }
}

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    /* System interrupt init*/
    /* PendSV_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    //initGPIO();
    // 1kHz ticks
    HAL_SYSTICK_Config(SystemCoreClock / 1000);

    timer.RegisterInterrupt(2, blink);
    USBCDC usb;

    for (;;)
        HAL_Delay(100); // __WFI(); // WFI causes some problems with the USB library?

    return 0;
}
