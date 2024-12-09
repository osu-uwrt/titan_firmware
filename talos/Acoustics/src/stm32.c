#include "stm32f4xx_hal.h"

#include <stdio.h>

#define ADC_THRESHOLD 3500  // Arbitrary threshold to detect peaks

// Variables for storing peak times and values
uint32_t last_peak_time_adc1 = 0;
uint32_t last_peak_time_adc2 = 0;
uint16_t last_value_adc1 = 0;
uint16_t last_value_adc2 = 0;

UART_HandleTypeDef huart1;  // UART handle for communication

// SysTick Handler to generate 1ms ticks
void SysTick_Handler(void) {
    HAL_IncTick();             // Increment HAL tick
    HAL_SYSTICK_IRQHandler();  // Call the HAL function to handle the SysTick interrupt
}

// Function to initialize UART (for printf)
void UART_Init(void) {
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK) {
        // Initialization error
        Error_Handler();
    }
}

// Function to print via UART (printf will use this)
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart1, (uint8_t *) &ch, 1, HAL_MAX_DELAY);
    return ch;
}

int main(void) {
    HAL_Init();  // Initialize the HAL library

    // Configure the system clock
    SystemClock_Config();

    // Initialize the UART
    UART_Init();

    // ADC initialization: Enable ADC and configure channels
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_ADC2_CLK_ENABLE();

    ADC_HandleTypeDef hadc1;
    ADC_HandleTypeDef hadc2;

    hadc1.Instance = ADC1;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    HAL_ADC_Init(&hadc1);

    hadc2.Instance = ADC2;
    hadc2.Init.Resolution = ADC_RESOLUTION_12B;
    hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion = 1;
    HAL_ADC_Init(&hadc2);

    // GPIO configuration (Analog pins PA0 and PA1)
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Start ADC conversions
    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc2);

    uint32_t current_time = 0;

    while (1) {
        // Read ADC values
        uint16_t raw_value_adc1 = HAL_ADC_GetValue(&hadc1);
        uint16_t raw_value_adc2 = HAL_ADC_GetValue(&hadc2);

        // Get current time from SysTick (in ms)
        current_time = HAL_GetTick();

        // Check for peak in ADC1
        if (raw_value_adc1 > last_value_adc1 && raw_value_adc1 > ADC_THRESHOLD) {
            last_value_adc1 = raw_value_adc1;
            last_peak_time_adc1 = current_time;
            printf("ADC1 Peak Value: %u, Time: %u ms\n", raw_value_adc1, last_peak_time_adc1);
        }

        // Check for peak in ADC2
        if (raw_value_adc2 > last_value_adc2 && raw_value_adc2 > ADC_THRESHOLD) {
            last_value_adc2 = raw_value_adc2;
            last_peak_time_adc2 = current_time;
            printf("ADC2 Peak Value: %u, Time: %u ms\n", raw_value_adc2, last_peak_time_adc2);
        }

        // If both peaks are detected, calculate the time difference
        if (last_peak_time_adc1 != 0 && last_peak_time_adc2 != 0) {
            int32_t time_diff = (int32_t) (last_peak_time_adc2 - last_peak_time_adc1);
            printf("Time difference between peaks: %d ms\n", time_diff);
        }

        HAL_Delay(50);  // Short delay before the next reading (adjust as needed)
    }
}

// Error handler function (for debugging purposes)
void Error_Handler(void) {
    while (1) {
        // Stay here in case of error
    }
}
