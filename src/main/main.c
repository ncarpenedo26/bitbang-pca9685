#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lib/manual_gpio.h"

#define LED_PIN 2

void app_main(void)
{
    gpio_enable(LED_PIN);
    for(;;) {
        gpio_set(LED_PIN);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_clear(LED_PIN);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
