#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#include "lib/manual_gpio.h"
#include "lib/pca9685.h"

#define LED_PIN 2
#define I2C_MASTER_SCL_IO           22      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21      /*!< GPIO number used for I2C master data  */

void app_main(void)
{
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x40,
        .scl_speed_hz = 100000,
    };

    pca9685_handle_t pca9685;
    ESP_ERROR_CHECK(pca9685_init(bus_handle, &dev_cfg, &pca9685));
    double count = 0;
    for(;;) {
        ESP_ERROR_CHECK(pca9685_set_all_duty_cycle(pca9685, count, 0) );
        vTaskDelay(100 / portTICK_PERIOD_MS);
        count += 0.01;
        if (count >= 1) {
            count = 0;
        };
    }
}