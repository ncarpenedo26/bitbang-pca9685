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

static const char *TAG = "pca9685";
static const uint32_t NUM_CHANNELS = 16;
static const uint32_t MAX_COUNT = 4095;
static const uint32_t INTERNAL_CLOCK_FREQ = 25e6;
static const int  DEFAULT_TIMEOUT = -1;

static const uint32_t PWM_FREQ_MAX = 1526;
static const uint32_t PWM_FREQ_MIN = 24;
static const uint32_t PRE_SCALE_MIN = 0x03;
static const uint32_t PRE_SCALE_MAX = 0xFF;

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

    // LEDK_ON/OFF_L/H omitted for LED1-LED15

    ALL_LED_ON_L = 0xFA,
    ALL_LED_ON_H = 0xFB,
    ALL_LED_OFF_L = 0xFC,
    ALL_LED_OFF_H = 0xFD,
    PRE_SCALE = 0xFE,
    TEST_MODE = 0x255,
} Reg;

typedef enum {
    RESTART = 1UL << 7,
    EXTCLK = 1UL << 6,
    AI = 1UL << 5,
    SLEEP = 1UL << 4,
    SUB1 = 1UL << 3,
    SUB2 = 1UL << 2,
    SUB3 = 1UL << 1,
    ALLCALL = 1UL,
} MODE1_FLAGS;

static const uint8_t DEFAULT_MODE1_CONFIG = AI | ALLCALL;

typedef enum {
    INVRT = 1UL << 4,
    OCH = 1UL << 3,
    OUTDRV = 1UL << 2,
    OUTNE = 1UL, // Bits [0:1]
} MODE2_FLAGS;


esp_err_t setup_pca9685(i2c_master_bus_handle_t bus_handle, const i2c_device_config_t *dev_config, pca9685_handle_t *ret_handle) {
    pca9685_dev_t *pca9685_dev = heap_caps_calloc(1, sizeof(pca9685_dev_t), MALLOC_CAP_DEFAULT);
    ESP_RETURN_ON_FALSE((bus_handle != NULL), ESP_ERR_NO_MEM, TAG, "insufficient memory for pca9685");
    esp_err_t err __attribute__((unused));
    err = i2c_master_bus_add_device(bus_handle, dev_config, &pca9685_dev->i2c_handle);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to add pca9685 to i2c bus");
    
    *ret_handle = pca9685_dev;
    
    uint8_t init_data[2] = {MODE1, DEFAULT_MODE1_CONFIG};
    err = i2c_master_transmit(pca9685_dev->i2c_handle, init_data, 2, DEFAULT_TIMEOUT);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to configure pca9685");

    return ESP_OK;
}

static esp_err_t i2c_read_reg(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t* reg_val) {
    // Write to the register
    esp_err_t err __attribute__((unused));
    err = i2c_master_transmit(handle, &reg, 1, DEFAULT_TIMEOUT);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to reach pca9685");

    // Read back the register
    err = i2c_master_receive(handle, reg_val, 1, DEFAULT_TIMEOUT);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to reach pca9685");
    
    return ESP_OK;
}

// The PRE_SCALE register can only be set when the SLEEP bit of MODE1 register is set to logic 1.
static uint8_t freq_to_prescale(uint32_t freq) {
    assert(freq <= PWM_FREQ_MAX);
    assert(freq >= PWM_FREQ_MIN);
    return INTERNAL_CLOCK_FREQ / (MAX_COUNT * freq) - 1;
}

static uint32_t prescale_to_freq(uint8_t prescale) {
    assert(prescale >= PRE_SCALE_MIN);
    assert(prescale <= PRE_SCALE_MAX);
    return INTERNAL_CLOCK_FREQ / (MAX_COUNT * (prescale + 1));
}

esp_err_t pca9685_get_prescale(pca9685_handle_t handle, uint8_t* prescale) {
    esp_err_t err __attribute__((unused));
    err = i2c_read_reg(handle->i2c_handle, PRE_SCALE, prescale);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to reach pca9685");
    return ESP_OK;
}

esp_err_t pca9685_get_freq(pca9685_handle_t handle, uint32_t* freq) {
    uint8_t prescale;
    esp_err_t err __attribute__((unused));
    err = pca9685_get_prescale(handle, &prescale);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to read prescale from pca9685");
    *freq = prescale_to_freq(prescale);
    return ESP_OK;
}

static uint8_t pca9685_channel_to_base_reg(uint32_t channel) {
    assert(channel < NUM_CHANNELS);
    return LED0_ON_L + (channel * 0x4);
}

static void get_counts_from_duty_cycle(double duty_cycle, double phase_delay, uint16_t* on_counts, uint16_t* off_counts) {
    assert(duty_cycle <= 1); 
    assert(duty_cycle >= 0); 
    assert(phase_delay <= 1);
    assert(phase_delay >= 0);

    *on_counts = (uint16_t)(phase_delay * MAX_COUNT) % 4096;
    *off_counts = (uint16_t)(duty_cycle * MAX_COUNT + phase_delay * MAX_COUNT) % 4096;
}

esp_err_t set_channel(pca9685_handle_t handle, uint32_t channel, uint16_t on_counts, uint16_t off_counts) {
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
    esp_err_t err __attribute__((unused));
    err = i2c_master_transmit(handle->i2c_handle, buffer, sizeof(buffer), -1);
    ESP_RETURN_ON_ERROR(err, TAG, "i2c master transmit failed");

    return ESP_OK;
}

esp_err_t set_channel_duty_cycle(pca9685_handle_t handle, uint32_t channel, double duty_cycle, double phase_delay) {
    ESP_RETURN_ON_FALSE(channel < NUM_CHANNELS, ESP_ERR_INVALID_ARG, TAG, "channel out of bounds");
    ESP_RETURN_ON_FALSE(duty_cycle <= 1, ESP_ERR_INVALID_ARG, TAG, "duty cycle must not be greater than 1");
    ESP_RETURN_ON_FALSE(duty_cycle >= 0, ESP_ERR_INVALID_ARG, TAG, "duty cycle must be positive");
    ESP_RETURN_ON_FALSE(phase_delay <= 1, ESP_ERR_INVALID_ARG, TAG, "phase delay must not be greater than 1");
    ESP_RETURN_ON_FALSE(phase_delay >= 0, ESP_ERR_INVALID_ARG, TAG, "phase delay must be positive");

    uint16_t on_counts, off_counts;
    get_counts_from_duty_cycle(duty_cycle, phase_delay, &on_counts, &off_counts);

    return set_channel(handle, channel, on_counts, off_counts);
}

esp_err_t set_channel_pulse_width(pca9685_handle_t handle, uint32_t channel, uint32_t pulse_width_us, uint32_t phase_shift_us) {
    uint32_t freq;
    esp_err_t err __attribute__((unused));
    err = pca9685_get_freq(handle, &freq);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to read pca9685 frequency");
    double period_us = (1 / ((double) freq)) * 1e6;

    ESP_RETURN_ON_FALSE(pulse_width_us <= period_us, ESP_ERR_INVALID_ARG, TAG, "pulse width must be less than PWM period.");
    ESP_RETURN_ON_FALSE(phase_shift_us <= period_us, ESP_ERR_INVALID_ARG, TAG, "phase shift must be less than PWM period.");

    double duty_cycle = ((double) pulse_width_us) / period_us;
    double phase_delay = ((double) phase_shift_us) / period_us;

    return set_channel_duty_cycle(handle, channel, duty_cycle, phase_delay);
}

esp_err_t set_channel_on(pca9685_handle_t handle, uint32_t channel) {
    ESP_RETURN_ON_FALSE(channel < NUM_CHANNELS, ESP_ERR_INVALID_ARG, TAG, "channel out of bounds");
    return set_channel(handle, channel, 1UL<<12, 0x0);
}

esp_err_t set_channel_off(pca9685_handle_t handle, uint32_t channel) {
    ESP_RETURN_ON_FALSE(channel < NUM_CHANNELS, ESP_ERR_INVALID_ARG, TAG, "channel out of bounds");
    return set_channel(handle, channel, 0x0, 1UL<<12);
}

esp_err_t set_all(pca9685_handle_t handle, uint16_t on_counts, uint16_t off_counts) {
    const uint8_t reg = ALL_LED_ON_L;

    uint8_t buffer[5] = {
        reg,                            // Starting register address (auto-increment enabled)
        (uint8_t) on_counts,            // ON low byte
        (uint8_t) (on_counts >> 8),     // ON high byte
        (uint8_t) off_counts,           // OFF low byte
        (uint8_t) (off_counts >> 8),    // OFF high byte
    };

    // Transmit all data in a single I2C transaction
    esp_err_t err __attribute__((unused));
    err = i2c_master_transmit(handle->i2c_handle, buffer, sizeof(buffer), -1);
    ESP_RETURN_ON_ERROR(err, TAG, "i2c master transmit failed");

    return ESP_OK;
}

esp_err_t set_all_on(pca9685_handle_t handle) {
    return set_all(handle, 1UL<<12, 0x0);
}

esp_err_t set_all_off(pca9685_handle_t handle) {
    return set_all(handle, 0x0, 1UL<<12);
}

esp_err_t set_all_duty_cycle(pca9685_handle_t handle, double duty_cycle, double phase_delay) {
    ESP_RETURN_ON_FALSE(duty_cycle <= 1, ESP_ERR_INVALID_ARG, TAG, "duty cycle must not be greater than 1");
    ESP_RETURN_ON_FALSE(duty_cycle >= 0, ESP_ERR_INVALID_ARG, TAG, "duty cycle must be positive");
    ESP_RETURN_ON_FALSE(phase_delay <= 1, ESP_ERR_INVALID_ARG, TAG, "phase delay must not be greater than 1");
    ESP_RETURN_ON_FALSE(phase_delay >= 0, ESP_ERR_INVALID_ARG, TAG, "phase delay must be positive");

    uint16_t on_counts, off_counts;
    get_counts_from_duty_cycle(duty_cycle, phase_delay, &on_counts, &off_counts);

    return set_all(handle, on_counts, off_counts);
}

esp_err_t set_all_pulse_width(pca9685_handle_t handle, uint32_t pulse_width_us, uint32_t phase_shift_us) {
    uint32_t freq;
    esp_err_t err __attribute__((unused));
    err = pca9685_get_freq(handle, &freq);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to read pca9685 frequency");
    double period_us = (1 / ((double) freq)) * 1e6;

    ESP_RETURN_ON_FALSE(pulse_width_us <= period_us, ESP_ERR_INVALID_ARG, TAG, "pulse width must be less than PWM period.");
    ESP_RETURN_ON_FALSE(phase_shift_us <= period_us, ESP_ERR_INVALID_ARG, TAG, "phase shift must be less than PWM period.");

    double duty_cycle = ((double) pulse_width_us) / period_us;
    double phase_delay = ((double) phase_shift_us) / period_us;

    return set_all_duty_cycle(handle, duty_cycle, phase_delay);
}

static esp_err_t set_bits(pca9685_handle_t handle, uint8_t reg, uint8_t mask) {
    uint8_t reg_status;
    esp_err_t err __attribute__((unused));
    err = i2c_read_reg(handle->i2c_handle, reg, &reg_status);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to read reg %x", reg);

    reg_status |= mask;
    uint8_t buffer[2] = {reg, reg_status};
    err = i2c_master_transmit(handle->i2c_handle, buffer, 2, DEFAULT_TIMEOUT);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to write bit %x of reg %x", mask, reg);

    return ESP_OK;
}

static esp_err_t clear_bits(pca9685_handle_t handle, uint8_t reg, uint8_t mask) {
    uint8_t reg_status;
    esp_err_t err __attribute__((unused));
    err = i2c_read_reg(handle->i2c_handle, reg, &reg_status);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to read reg %x", reg);

    reg_status &= ~mask;
    uint8_t buffer[2] = {reg, reg_status};
    err = i2c_master_transmit(handle->i2c_handle, buffer, 2, DEFAULT_TIMEOUT);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to write bit %x of reg %x", mask, reg);

    return ESP_OK;
}

esp_err_t pca9685_sleep(pca9685_handle_t handle) {
    return set_bits(handle, MODE1, SLEEP);
}

esp_err_t pca9685_wake(pca9685_handle_t handle) {
    return clear_bits(handle, MODE1, SLEEP);
}

esp_err_t pca9685_restart(pca9685_handle_t handle) {
    uint8_t mode1_status;
    esp_err_t err __attribute__((unused));

    err = i2c_read_reg(handle->i2c_handle, MODE1, &mode1_status);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to read pca9685");

    bool restart_logic_high = mode1_status & RESTART;
    if (!restart_logic_high) {
        return ESP_OK;
    }

    err = clear_bits(handle, MODE1, SLEEP);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to clear sleep bit");

    vTaskDelay(1 / portTICK_PERIOD_MS);
   
    err = set_bits(handle, MODE1, RESTART);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to restart pca9685");

    return ESP_OK;
}

esp_err_t pca9685_use_extclk(pca9685_handle_t handle) {
    esp_err_t err __attribute__((unused));

    err = set_bits(handle, MODE1, SLEEP);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to set bits");

    err = set_bits(handle, MODE1, SLEEP | EXTCLK);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to set bits");
    
    err = clear_bits(handle, MODE1, SLEEP);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to set bits");
    
    return ESP_OK;
}

// get/set output driver mode
// get/set output disabled mode
// get/set output enabled mode
// get/set channel update mode
// enable/disable all call address
// enable/disable sub123 addresses
// enable/disable ext clock line