#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* I2C */

static const char *TAG = "I2C-PWM";

#define I2C_MASTER_SCL_IO           22                         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21                         /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000
#define SAMPLE_RATE_MS              500

#define SENSOR_ADDR                 0x4D                       /*!< Slave address of the sensor */

/* LEDC control */

#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       18
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0

#define LEDC_TEST_CH_NUM       4
#define LEDC_TEST_DUTY         4000
#define LEDC_TEST_FADE_TIME    500


/**
 * Read a sequence of bytes from sensor
 */
static esp_err_t sensor_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_read_from_device(I2C_MASTER_NUM, SENSOR_ADDR, data, \
    len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

/**
 * I2C master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, \
     I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/*
 * This callback function will be called when fade operation has ended
 * Use callback only if you are aware it is being called inside an ISR
 * Otherwise, you can use a semaphore to unblock tasks
 */
static bool cb_ledc_fade_end_event(const ledc_cb_param_t *param, void *user_arg)
{
    portBASE_TYPE taskAwoken = pdFALSE;

    if (param->event == LEDC_FADE_END_EVT) {
        SemaphoreHandle_t counting_sem = (SemaphoreHandle_t) user_arg;
        xSemaphoreGiveFromISR(counting_sem, &taskAwoken);
    }

    return (taskAwoken == pdTRUE);
}

void app_main(void)
{   
    /* I2C */

    uint8_t data;
    

    /* LEDC control */

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT,   // resolution of PWM duty
        .freq_hz = 1000,                        // frequency of PWM signal
        .speed_mode = LEDC_HS_MODE,             // timer mode
        .timer_num = LEDC_HS_TIMER,             // timer index
        .clk_cfg = LEDC_AUTO_CLK,               // Auto select the source clock
    };

    
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_HS_CH0_CHANNEL,
        .duty       = 0,
        .gpio_num   = LEDC_HS_CH0_GPIO,
        .speed_mode = LEDC_HS_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_HS_TIMER,
        .flags.output_invert = 0
    };

    // Set LED Controller with previously prepared configuration

    ledc_channel_config(&ledc_channel);


    // Initialize fade service.
    ledc_fade_func_install(0);
    ledc_cbs_t callbacks = {
        .fade_cb = cb_ledc_fade_end_event
    };
    SemaphoreHandle_t counting_sem = xSemaphoreCreateCounting(LEDC_TEST_CH_NUM, 0);
    ledc_cb_register(ledc_channel.speed_mode, ledc_channel.channel, &callbacks, (void *) counting_sem);

    ESP_ERROR_CHECK(i2c_master_init());

    int duty;

    while (1) {

        ESP_ERROR_CHECK(sensor_read(SENSOR_ADDR, &data, 1));
        ESP_LOGI(TAG, "Val = %d", data);

        duty = 0;

        if(data >= 24){
            duty = ((23 - data) * -1) * 511;

            ledc_set_fade_with_time(ledc_channel.speed_mode,ledc_channel.channel, duty, LEDC_TEST_FADE_TIME);
            ledc_fade_start(ledc_channel.speed_mode,ledc_channel.channel, LEDC_FADE_NO_WAIT);
            
            xSemaphoreTake(counting_sem, portMAX_DELAY);
            
        }
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
    }

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
}