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
static const uint32_t INTERNAL_CLOCK_FREQ = 25e6;

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

    PRE_SCALE = 0xFE,
    // TODO: Add more registers
} Reg;

esp_err_t setup_pca9685(i2c_master_bus_handle_t bus_handle, const i2c_device_config_t *dev_config, pca9685_handle_t *ret_handle) {
    pca9685_dev_t *pca9685_dev = heap_caps_calloc(1, sizeof(pca9685_dev_t), MALLOC_CAP_DEFAULT);
    ESP_RETURN_ON_FALSE((bus_handle != NULL), ESP_ERR_NO_MEM, TAG, "insufficient memory for pca9685");
    esp_err_t success = i2c_master_bus_add_device(bus_handle, dev_config, &pca9685_dev->i2c_handle);
    *ret_handle = pca9685_dev;
    
    uint8_t init_data[2] = {MODE1, 0x20}; // MODE1: Normal mode (no sleep)
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

// The PRE_SCALE register can only be set when the SLEEP bit of MODE1 register is set to logic 1.
static uint8_t freq_to_prescale(uint8_t freq) {
    // TODO: error handle instead of clamp
    return INTERNAL_CLOCK_FREQ / (MAX_COUNT * freq) - 1;
}

static uint8_t prescale_to_freq(uint8_t prescale) {
    // TODO: error handle instead of clamp
    return INTERNAL_CLOCK_FREQ / (MAX_COUNT * (prescale + 1));
}

esp_err_t pca9685_get_prescale(pca9685_handle_t handle, uint8_t* prescale) {
    *prescale = i2c_read_reg(handle->i2c_handle, PRE_SCALE);
    return ESP_OK; // TODO: Actual error handling
}

esp_err_t pca9685_get_freq(pca9685_handle_t handle, uint8_t* freq) {
    uint8_t prescale;
    esp_err_t err = pca9685_get_prescale(handle, &prescale);
    ESP_RETURN_ON_FALSE(err == ESP_OK, err, TAG, "failed to read prescale from pca9685");
    *freq = prescale_to_freq(prescale);
    return ESP_OK;
}

// TODO: better error handling
static uint8_t pca9685_channel_to_base_reg(uint32_t channel) {
    return LED0_ON_L + (channel * 0x4);
}

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

    const uint8_t reg = pca9685_channel_to_base_reg(channel);

    uint8_t buffer[5] = {
        reg,                            // Starting register address (auto-increment enabled)
        (uint8_t) on_counts,            // ON low byte
        (uint8_t) (on_counts >> 8),     // ON high byte
        (uint8_t) off_counts,           // OFF low byte
        (uint8_t) (off_counts >> 8),    // OFF high byte
    };

    // Transmit all data in a single I2C transaction
    i2c_master_transmit(handle->i2c_handle, buffer, sizeof(buffer), -1);


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

esp_err_t set_channel_pulse_width(pca9685_handle_t handle, uint32_t channel, uint32_t pulse_width_us, uint32_t phase_shift_us) {

    uint8_t freq;
    esp_err_t err = pca9685_get_freq(handle, &freq);
    double period_us = (1 / ((double) freq)) * 1e6;
    ESP_RETURN_ON_FALSE(pulse_width_us <= period_us, ESP_ERR_INVALID_ARG, TAG, "pulse width must be less than PWM period.");
    ESP_RETURN_ON_FALSE(phase_shift_us <= period_us, ESP_ERR_INVALID_ARG, TAG, "phase shift must be less than PWM period.");

    double duty_cycle = ((double) pulse_width_us) / period_us;
    double phase_delay = ((double) phase_shift_us) / period_us;

    return set_channel_duty_cycle(handle, channel, duty_cycle, phase_delay);
}
