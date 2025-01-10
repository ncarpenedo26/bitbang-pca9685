#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bitbang_i2c.h"


#include "driver/gpio.h"
#include "soc/gpio_reg.h"

#define LED_PIN 2

void app_main(void)
{
    uint32_t bitmask = 1UL << LED_PIN;
    *((volatile uint32_t *)GPIO_ENABLE_REG) |= (uint32_t) bitmask;
    gpio_enable(LED_PIN);
    for(;;) {
        gpio_set(LED_PIN);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_clear(LED_PIN);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
