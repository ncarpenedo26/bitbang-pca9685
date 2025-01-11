#include "pca9685.h"

#include "driver/i2c_slave.h"
#include "driver/i2c_master.h"
#include "soc/gpio_reg.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "esp_memory_utils.h"
#include "freertos/idf_additions.h"
#include "esp_types.h"
#include "esp_attr.h"
#include "esp_check.h"

#define DATA_LENGTH 100

static const char *TAG = "pca9685";
static const uint32_t NUM_CHANNELS = 16;
static const uint32_t MAX_COUNT = 4095;

typedef enum {
    MODE1 = 0x00,
    MODE2 = 0x01,
    SUBADR1 = 0x02,
    SUBADR2 = 0x03,
    SUBADR3 = 0x04,
    ALLCALLADR = 0x05,
    LED0_ON_L = 0x06,
    LED0_ON_H = 0x07,
    LED0_OFF_L = 0x08,
    LED0_OFF_H = 0x09,
    // TODO: Add more registers
} Reg;

esp_err_t setup_pca9685(i2c_master_bus_handle_t bus_handle, const i2c_device_config_t *dev_config, pca9685_handle_t *ret_handle) {
    pca9685_dev_t *pca9685_dev = heap_caps_calloc(1, sizeof(pca9685_dev_t), MALLOC_CAP_DEFAULT);
    ESP_RETURN_ON_FALSE((bus_handle != NULL), ESP_ERR_NO_MEM, TAG, "insufficient memory for pca9685");
    esp_err_t success = i2c_master_bus_add_device(bus_handle, dev_config, &pca9685_dev->i2c_handle);
    *ret_handle = pca9685_dev;
    
    uint8_t init_data[2] = {MODE1, 0x00}; // MODE1: Normal mode (no sleep)
    esp_err_t err = i2c_master_transmit(pca9685_dev->i2c_handle, init_data, 2, -1);
    if (err != ESP_OK) {
        printf("Failed to initialize MODE1: %d\n", err);
        return err;
    }

    return success;
}

// TODO: Add error checking
static uint8_t i2c_read_reg(i2c_master_dev_handle_t handle, uint8_t reg) {
    uint8_t ret;

    // Write to the register
    i2c_master_transmit(handle, &reg, 1, -1);

    // Read back the register
    i2c_master_receive(handle, &ret, 1, -1);
    return ret;
}

// static uint8_t pca9685_channel_to_reg() {

// }

esp_err_t get_counts_from_duty_cycle(double duty_cycle, double phase_delay, uint16_t* on_counts, uint16_t* off_counts) {
    ESP_RETURN_ON_FALSE(duty_cycle <= 1, ESP_ERR_INVALID_ARG, TAG, "duty cycle must not be greater than 1");
    ESP_RETURN_ON_FALSE(duty_cycle >= 0, ESP_ERR_INVALID_ARG, TAG, "duty cycle must be positive");
    ESP_RETURN_ON_FALSE(phase_delay <= 1, ESP_ERR_INVALID_ARG, TAG, "phase delay must not be greater than 1");
    ESP_RETURN_ON_FALSE(phase_delay >= 0, ESP_ERR_INVALID_ARG, TAG, "phase delay must be positive");
    *on_counts = (uint16_t)(phase_delay * MAX_COUNT) % 4096;
    *off_counts = (uint16_t)(duty_cycle * MAX_COUNT + phase_delay * MAX_COUNT) % 4096;
    return ESP_OK;
}

// TODO: add more error checking
// TODO: figure out proper behavior of high bit 4
esp_err_t set_channel(pca9685_handle_t handle, uint32_t channel, uint16_t on_counts, uint16_t off_counts) {
    ESP_RETURN_ON_FALSE(on_counts <= 0xfff, ESP_ERR_INVALID_ARG, TAG, "on_counts too large");
    ESP_RETURN_ON_FALSE(off_counts <= 0xfff, ESP_ERR_INVALID_ARG, TAG, "off_counts too large");
    ESP_RETURN_ON_FALSE(channel < NUM_CHANNELS, ESP_ERR_INVALID_ARG, TAG, "channel out of bounds");

    // TODO: Handle register access more gracefully
    uint8_t on_low[2] = {LED0_ON_L + channel*4, (uint8_t) on_counts};
    uint8_t on_high[2] = {LED0_ON_H + channel*4, (uint8_t) (on_counts>>8)};
    uint8_t off_low[2] = {LED0_OFF_L + channel*4, (uint8_t) off_counts};
    uint8_t off_high[2] = {LED0_OFF_H + channel*4, (uint8_t) (off_counts>>8)};

    i2c_master_transmit(handle->i2c_handle, on_low, 2, -1);
    i2c_master_transmit(handle->i2c_handle, on_high, 2, -1);

    i2c_master_transmit(handle->i2c_handle, off_low, 2, -1);
    i2c_master_transmit(handle->i2c_handle, off_high, 2, -1);

    return ESP_OK;
}

esp_err_t set_channel_duty_cycle(pca9685_handle_t handle, uint32_t channel, double duty_cycle, double phase_delay) {
    ESP_RETURN_ON_FALSE(channel < NUM_CHANNELS, ESP_ERR_INVALID_ARG, TAG, "channel out of bounds");
    ESP_RETURN_ON_FALSE(duty_cycle <= 1, ESP_ERR_INVALID_ARG, TAG, "duty cycle must not be greater than 1");
    ESP_RETURN_ON_FALSE(duty_cycle >= 0, ESP_ERR_INVALID_ARG, TAG, "duty cycle must be positive");
    ESP_RETURN_ON_FALSE(phase_delay <= 1, ESP_ERR_INVALID_ARG, TAG, "phase delay must not be greater than 1");
    ESP_RETURN_ON_FALSE(phase_delay >= 0, ESP_ERR_INVALID_ARG, TAG, "phase delay must be positive");

    uint16_t on_counts, off_counts;
    esp_err_t success = get_counts_from_duty_cycle(duty_cycle, phase_delay, &on_counts, &off_counts);
    ESP_RETURN_ON_FALSE(success == ESP_OK, success, TAG, "failed to calculate cycle counts");
    
    return set_channel(handle, channel, on_counts, off_counts);
}

