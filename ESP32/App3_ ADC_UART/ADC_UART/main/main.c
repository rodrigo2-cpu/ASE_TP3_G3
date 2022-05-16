#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "esp_log.h"

#define SAMPLE_CNT 32
#define BUF_SIZE 1024

#define SEC_MULTIPLIER 1000000l

static const char *TAG = "ADC_UART";

/* Configure adc channel */
static const adc1_channel_t adc_channel = ADC_CHANNEL_6;    //GPIO34 for ADC1


uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB
};

void setup() {
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);
}

static void init_hw(void)
{
    adc1_config_width(ADC_WIDTH_BIT_10);
    adc1_config_channel_atten(adc_channel, ADC_ATTEN_DB_11);
}

void app_main()
{
    init_hw();
    setup();

    char *buff = (char *)malloc(BUF_SIZE);

    uint32_t adc_val = 0;

    while (1)
    {
        /* Configure the timer and the touchpad as wakeup sources */
        esp_sleep_enable_timer_wakeup(4 * SEC_MULTIPLIER);
        esp_sleep_enable_touchpad_wakeup();

        /* Enter in ligh sleep mode*/
        esp_light_sleep_start();

        printf("Active at timer value: %lli \n", esp_timer_get_time() / SEC_MULTIPLIER);

        for (int i = 0; i < SAMPLE_CNT; ++i)
        {
            /* Read the value from adc */
            adc_val += adc1_get_raw(adc_channel);
        }
        
        adc_val /= SAMPLE_CNT;
        
        buff = (char*) adc_val;
       
        // Read data from the UART
        int len = uart_read_bytes(UART_NUM_0, buff, (BUF_SIZE - 1), 20 / portTICK_RATE_MS);
        
        // Write data back to the UART
        uart_write_bytes(UART_NUM_0, (char*) adc_val, len );
        
        ESP_LOGI(TAG, "Voltage: %d", (int) buff);
        
        vTaskDelay(500 / portTICK_RATE_MS);
    }
}