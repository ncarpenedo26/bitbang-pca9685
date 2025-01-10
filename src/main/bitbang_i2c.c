#include "driver/gpio.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/gpio_reg.h"
#include "bitbang_i2c.h"

inline void gpio_enable(uint32_t pin) {
    printf("GPIO REG: %lx\n", *((volatile uint32_t *)GPIO_ENABLE_REG));
    uint32_t bitmask = 1UL << pin;
    *((volatile uint32_t *)GPIO_ENABLE_REG) |= (uint32_t) bitmask;
    printf("GPIO REG: %lx\n", *((volatile uint32_t *)GPIO_ENABLE_REG));
}

void gpio_set(uint32_t pin) {
    printf("GPIO REG: %lx\n", *((volatile uint32_t *)GPIO_ENABLE_REG));
    uint32_t bitmask = 1UL << pin;
    volatile uint32_t* gpio_reg = (uint32_t*) GPIO_OUT_REG;
    *gpio_reg |= bitmask;
    printf("GPIO REG: %lx\n", *((volatile uint32_t *)GPIO_ENABLE_REG));
}

void gpio_clear(uint32_t pin) {
    printf("GPIO REG: %lx\n", *((volatile uint32_t *)GPIO_ENABLE_REG));
    uint32_t bitmask = 1UL << pin;
    volatile uint32_t* gpio_reg = (uint32_t*) GPIO_OUT_REG;
    *gpio_reg &= ~bitmask;
    printf("GPIO REG: %lx\n", *((volatile uint32_t *)GPIO_ENABLE_REG));
}