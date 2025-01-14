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

#define NUM_CHANNELS 16
#define MAX_COUNT 4095
#define INTERNAL_CLOCK_FREQ 25e6
#define DEFAULT_TIMEOUT -1

#define PWM_FREQ_MAX 1526
#define PWM_FREQ_MIN 24
#define PRE_SCALE_MIN 0x03
#define PRE_SCALE_MAX 0xFF

static const char *TAG = "pca9685";

/* I2C Register Addresses for PCA9685 */
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
} PCA9685_REG;

// Bit flags for MODE1 register
typedef enum {
    RESTART = 1UL << 7,
    EXTCLK = 1UL << 6,
    AI = 1UL << 5,
    SLEEP = 1UL << 4,
    SUB1 = 1UL << 3,
    SUB2 = 1UL << 2,
    SUB3 = 1UL << 1,
    ALLCALL = 1UL,
} MODE1_BITS;

// Bit flags for MODE2 register
typedef enum {
    INVRT = 1UL << 4,
    OCH = 1UL << 3,
    OUTDRV = 1UL << 2,
    OUTNE_H = 1UL << 1,
    OUTNE_L = 1UL,
} MODE2_BITS;

static const uint8_t DEFAULT_MODE1_CONFIG = AI | ALLCALL;

/** Reads contents of register into reg_val
 * @param handle PCA9685 device handle (obtained from pca9685_init())
 * @param reg i2c register address to read from PCA9685
 * @param reg_val contents of reg read into this address
 * @return success or failure
 */
static esp_err_t i2c_read_reg(i2c_master_dev_handle_t handle, const uint8_t reg, uint8_t* reg_val) {
    // Write to the register
    esp_err_t err __attribute__((unused));
    err = i2c_master_transmit(handle, &reg, 1, DEFAULT_TIMEOUT);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to reach pca9685");

    // Read back the register
    err = i2c_master_receive(handle, reg_val, 1, DEFAULT_TIMEOUT);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to reach pca9685");
    
    return ESP_OK;
}

/** Sets bits of register reg to logic high based on mask
 * @param handle PCA9685 device handle (obtained from pca9685_init())
 * @param reg i2c register address to read from PCA9685
 * @param mask which bits to set
 * @return success or failure
 */
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

/** Sets clears of register reg to logic low (0) based on mask
 * @param handle PCA9685 device handle (obtained from pca9685_init())
 * @param reg i2c register address to read from PCA9685
 * @param mask which bits to clear
 * @return success or failure
 */
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

/** Converts freq (hz) to prescale value. Must be within range 24hz to 1526 hz
 * @param freq frequency in hz
 * @returns prescale value scalar
 */
static uint8_t freq_to_prescale(uint32_t freq) {
    assert(freq <= PWM_FREQ_MAX);
    assert(freq >= PWM_FREQ_MIN);
    return INTERNAL_CLOCK_FREQ / (MAX_COUNT * freq) - 1;
}

/** Converts prescale value to frequency (hz)
 * @param prescale
 * @returns frequency in hz
 */
static uint32_t prescale_to_freq(uint8_t prescale) {
    assert(prescale >= PRE_SCALE_MIN);
    assert(prescale <= PRE_SCALE_MAX);
    return INTERNAL_CLOCK_FREQ / (MAX_COUNT * (prescale + 1));
}

/** Returns led base register address for channel
 * @param channel channel to get base reg for
 * @return register base address 
 */
static uint8_t pca9685_channel_to_base_reg(uint32_t channel) {
    assert(channel < NUM_CHANNELS);
    return LED0_ON_L + (channel * 0x4);
}

/** Converts duty_cycle and phase_delay to on and off count values
 *  @param duty_cycle as a percentage (double between 0 and 1)
 *  @param phase_delay as a percentage (double between 0 and 1) 
 *  @param on_counts address to write on_counts into (mutates)
 *  @param off_counts address to write off_counts into (mutates)
 */
static void get_counts_from_duty_cycle(double duty_cycle, double phase_delay, uint16_t* on_counts, uint16_t* off_counts) {
    assert(duty_cycle <= 1); 
    assert(duty_cycle >= 0); 
    assert(phase_delay <= 1);
    assert(phase_delay >= 0);

    *on_counts = (uint16_t)(phase_delay * MAX_COUNT) % 4096;
    *off_counts = (uint16_t)(duty_cycle * MAX_COUNT + phase_delay * MAX_COUNT) % 4096;
}

/** Initializes PCA9685 device
 * @param bus_handle i2c bus handle to attach to
 * @param dev_config pointer to i2c device config
 * @param ret_handle pca9685 handle to write to
 * @return success or failure
 */
esp_err_t pca9685_init(i2c_master_bus_handle_t bus_handle, const i2c_device_config_t *dev_config, pca9685_handle_t *ret_handle) {
    pca9685_dev_t *pca9685_dev = heap_caps_calloc(1, sizeof(pca9685_dev_t), MALLOC_CAP_DEFAULT);
    ESP_RETURN_ON_FALSE((bus_handle != NULL), ESP_ERR_NO_MEM, TAG, "insufficient memory for pca9685");

    esp_err_t err __attribute__((unused));
    err = i2c_master_bus_add_device(bus_handle, dev_config, &pca9685_dev->i2c_handle);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to add pca9685 to i2c bus");
    
    uint8_t init_data[2] = {MODE1, DEFAULT_MODE1_CONFIG};
    err = i2c_master_transmit(pca9685_dev->i2c_handle, init_data, 2, DEFAULT_TIMEOUT);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to configure pca9685");

    *ret_handle = pca9685_dev;
    
    return ESP_OK;
}

/** Get prescale value of PCA9685
 *  @param handle pca9685 device handle 
 *  @param prescale pointer to write prescale value into
 *  @return success or failure
 */
esp_err_t pca9685_get_prescale(pca9685_handle_t handle, uint8_t* prescale) {
    esp_err_t err __attribute__((unused));
    err = i2c_read_reg(handle->i2c_handle, PRE_SCALE, prescale);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to reach pca9685");
    return ESP_OK;
}

/** Set prescale value of PCA9685
 *  @param handle pca9685 device handle 
 *  @param prescale prescale value to set to
 *  @return success or failure
 */
esp_err_t pca9685_set_prescale(pca9685_handle_t handle, uint8_t prescale) {
    uint8_t buffer[2] = {PRE_SCALE, prescale};
    return i2c_master_transmit(handle->i2c_handle, buffer, 2, DEFAULT_TIMEOUT);
}

/** Get PWM frequency of PCA9685
 *  @param handle pca9685 device handle 
 *  @param freq pointer to write frequency value into
 *  @return success or failure
 */
esp_err_t pca9685_get_freq(pca9685_handle_t handle, uint32_t* freq) {
    uint8_t prescale;
    esp_err_t err __attribute__((unused));
    err = pca9685_get_prescale(handle, &prescale);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to read prescale from pca9685");
    *freq = prescale_to_freq(prescale);
    return ESP_OK;
}

/** Set set frequency value of PCA9685
 *  @param handle pca9685 device handle 
 *  @param freq frequency value to set to
 *  @return success or failure
 */
esp_err_t pca9685_set_freq(pca9685_handle_t handle, uint32_t freq) {
    return pca9685_set_prescale(handle, freq_to_prescale(freq));
}

/** Write PWM counts to base register
 *  @param handle device handle
 *  @param reg base register for LED channel
 *  @param on_counts counts to set
 *  @param off_counts counts to set
 *  @returns success or failure 
 */
static esp_err_t i2c_write_counts_to_reg(pca9685_handle_t handle, PCA9685_REG reg, uint16_t on_counts, uint16_t off_counts) {
    uint8_t buffer[5] = {
        reg,                            // Starting register address (auto-increment enabled)
        (uint8_t) on_counts,            // ON low byte
        (uint8_t) (on_counts >> 8),     // ON high byte
        (uint8_t) off_counts,           // OFF low byte
        (uint8_t) (off_counts >> 8),    // OFF high byte
    };

    // Transmit all data in a single I2C transaction
    return i2c_master_transmit(handle->i2c_handle, buffer, sizeof(buffer), -1);
}

/** Write PWM counts to channel
 *  @param handle device handle
 *  @param channel channel num 0-15 to set
 *  @param on_counts counts to set
 *  @param off_counts counts to set
 *  @returns success or failure 
 */
static esp_err_t set_channel(pca9685_handle_t handle, uint32_t channel, uint16_t on_counts, uint16_t off_counts) {
    ESP_RETURN_ON_FALSE(channel < NUM_CHANNELS, ESP_ERR_INVALID_ARG, TAG, "channel out of bounds");
    const uint8_t reg = pca9685_channel_to_base_reg(channel);
    return i2c_write_counts_to_reg(handle, reg, on_counts, off_counts);
}

/** Set channel duty cycle
 *  @param handle device handle
 *  @param channel channel num 0-15 to set
 *  @param on_counts duty cycle to set
 *  @param off_counts phase delay to set
 *  @returns success or failure 
 */
esp_err_t pca9685_set_duty_cycle(pca9685_handle_t handle, uint32_t channel, double duty_cycle, double phase_delay) {
    ESP_RETURN_ON_FALSE(channel < NUM_CHANNELS, ESP_ERR_INVALID_ARG, TAG, "channel out of bounds");
    ESP_RETURN_ON_FALSE(duty_cycle <= 1, ESP_ERR_INVALID_ARG, TAG, "duty cycle must not be greater than 1");
    ESP_RETURN_ON_FALSE(duty_cycle >= 0, ESP_ERR_INVALID_ARG, TAG, "duty cycle must be positive");
    ESP_RETURN_ON_FALSE(phase_delay <= 1, ESP_ERR_INVALID_ARG, TAG, "phase delay must not be greater than 1");
    ESP_RETURN_ON_FALSE(phase_delay >= 0, ESP_ERR_INVALID_ARG, TAG, "phase delay must be positive");

    uint16_t on_counts, off_counts;
    get_counts_from_duty_cycle(duty_cycle, phase_delay, &on_counts, &off_counts);

    return set_channel(handle, channel, on_counts, off_counts);
}

/** Set channel pulse width
 *  @param handle device handle
 *  @param channel channel num 0-15 to set
 *  @param on_counts pulse width to set (us)
 *  @param off_counts phase shift to set (us)
 *  @returns success or failure 
 */
esp_err_t pca9685_set_pulse_width(pca9685_handle_t handle, uint32_t channel, uint32_t pulse_width_us, uint32_t phase_shift_us) {
    uint32_t freq;
    esp_err_t err __attribute__((unused));
    err = pca9685_get_freq(handle, &freq);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to read pca9685 frequency");

    double period_us = (1 / ((double) freq)) * 1e6;

    ESP_RETURN_ON_FALSE(pulse_width_us <= period_us, ESP_ERR_INVALID_ARG, TAG, "pulse width must be less than PWM period.");
    ESP_RETURN_ON_FALSE(phase_shift_us <= period_us, ESP_ERR_INVALID_ARG, TAG, "phase shift must be less than PWM period.");

    double duty_cycle = ((double) pulse_width_us) / period_us;
    double phase_delay = ((double) phase_shift_us) / period_us;

    return pca9685_set_duty_cycle(handle, channel, duty_cycle, phase_delay);
}

/** Set channel full on
 *  @param handle device handle
 *  @param channel channel num 0-15 to set
 *  @returns success or failure 
 */
esp_err_t pca9685_set_channel_on(pca9685_handle_t handle, uint32_t channel) {
    ESP_RETURN_ON_FALSE(channel < NUM_CHANNELS, ESP_ERR_INVALID_ARG, TAG, "channel out of bounds");
    return set_channel(handle, channel, 1UL<<12, 0x0);
}

/** Set channel full off
 *  @param handle device handle
 *  @param channel channel num 0-15 to set
 *  @returns success or failure 
 */
esp_err_t pca9685_set_channel_off(pca9685_handle_t handle, uint32_t channel) {
    ESP_RETURN_ON_FALSE(channel < NUM_CHANNELS, ESP_ERR_INVALID_ARG, TAG, "channel out of bounds");
    return set_channel(handle, channel, 0x0, 1UL<<12);
}

/** Write PWM counts to all channels
 *  @param handle device handle
 *  @param on_counts counts to set
 *  @param off_counts counts to set
 *  @returns success or failure 
 */
static esp_err_t set_all(pca9685_handle_t handle, uint16_t on_counts, uint16_t off_counts) {
    return i2c_write_counts_to_reg(handle, ALL_LED_ON_L, on_counts, off_counts);
}

/** Set channel full on
 *  @param handle device handle
 *  @returns success or failure 
 */
esp_err_t pca9685_set_all_on(pca9685_handle_t handle) {
    return set_all(handle, 1UL<<12, 0x0);
}

/** Set channel full off
 *  @param handle device handle
 *  @returns success or failure 
 */
esp_err_t pca9685_set_all_off(pca9685_handle_t handle) {
    return set_all(handle, 0x0, 1UL<<12);
}

/** Set all channels duty cycle
 *  @param handle device handle
 *  @param on_counts duty cycle to set
 *  @param off_counts phase delay to set
 *  @returns success or failure 
 */
esp_err_t pca9685_set_all_duty_cycle(pca9685_handle_t handle, double duty_cycle, double phase_delay) {
    ESP_RETURN_ON_FALSE(duty_cycle <= 1, ESP_ERR_INVALID_ARG, TAG, "duty cycle must not be greater than 1");
    ESP_RETURN_ON_FALSE(duty_cycle >= 0, ESP_ERR_INVALID_ARG, TAG, "duty cycle must be positive");
    ESP_RETURN_ON_FALSE(phase_delay <= 1, ESP_ERR_INVALID_ARG, TAG, "phase delay must not be greater than 1");
    ESP_RETURN_ON_FALSE(phase_delay >= 0, ESP_ERR_INVALID_ARG, TAG, "phase delay must be positive");

    uint16_t on_counts, off_counts;
    get_counts_from_duty_cycle(duty_cycle, phase_delay, &on_counts, &off_counts);

    return set_all(handle, on_counts, off_counts);
}

/** Set all channels pulse width
 *  @param handle device handle
 *  @param on_counts pulse width to set (us)
 *  @param off_counts phase shift to set (us)
 *  @returns success or failure 
 */
esp_err_t pca9685_set_all_pulse_width(pca9685_handle_t handle, uint32_t pulse_width_us, uint32_t phase_shift_us) {
    uint32_t freq;
    esp_err_t err __attribute__((unused));
    err = pca9685_get_freq(handle, &freq);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to read pca9685 frequency");
    double period_us = (1 / ((double) freq)) * 1e6;

    ESP_RETURN_ON_FALSE(pulse_width_us <= period_us, ESP_ERR_INVALID_ARG, TAG, "pulse width must be less than PWM period.");
    ESP_RETURN_ON_FALSE(phase_shift_us <= period_us, ESP_ERR_INVALID_ARG, TAG, "phase shift must be less than PWM period.");

    double duty_cycle = ((double) pulse_width_us) / period_us;
    double phase_delay = ((double) phase_shift_us) / period_us;

    return pca9685_set_all_duty_cycle(handle, duty_cycle, phase_delay);
}

/** Put the device to sleep / low power mode
 *  @param handle device handle
 *  @returns success or failure 
 */
esp_err_t pca9685_disable(pca9685_handle_t handle) {
    return set_bits(handle, MODE1, SLEEP);
}

/** Wake up the device from low power sleep
 *  @param handle device handle
 *  @returns success or failure 
 */
esp_err_t pca9685_enable(pca9685_handle_t handle) {
    return clear_bits(handle, MODE1, SLEEP);
}

/** Restart the device from sleep, resetting PWM values to what they were
 *  before sleeping.
 *  @param handle device handle
 *  @returns success or failure 
 */
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

/** Switch from internal clock to external clock via EXTCLK pin.
 *  Can only be cleared by resetting the device.
 *  @param handle device handle
 *  @returns success or failure 
 */
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

/** Configure PCA9685 to respond to specified all-call address
 *  @param handle device handle
 *  @param all_call_addr i2c address to respond to
 *  @returns success or failure 
 */
esp_err_t pca9685_enable_all_call(pca9685_handle_t handle, uint8_t all_call_addr) {
    esp_err_t err __attribute__((unused));

    err = set_bits(handle, MODE1, ALLCALL);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to set bits");

    uint8_t buffer[2] = {ALLCALLADR, all_call_addr};
    err = i2c_master_transmit(handle->i2c_handle, buffer, 2, DEFAULT_TIMEOUT);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to write address");

    return ESP_OK;
}

/** Configure PCA9685 to not respond to all-call address
 *  @param handle device handle
 *  @returns success or failure 
 */
esp_err_t pca9685_disable_all_call(pca9685_handle_t handle) {
    return clear_bits(handle, MODE1, ALLCALL);
}

/** Configure PCA9685 to respond to specified subaddress
 *  @param handle device handle
 *  @param all_call_addr i2c address to respond to
 *  @returns success or failure 
 */
esp_err_t pca9685_enable_sub1(pca9685_handle_t handle, uint8_t sub1_addr) {
    esp_err_t err __attribute__((unused));

    err = set_bits(handle, MODE1, SUB1);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to set bits");

    uint8_t buffer[2] = {SUBADR1, sub1_addr};
    err = i2c_master_transmit(handle->i2c_handle, buffer, 2, DEFAULT_TIMEOUT);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to write address");

    return ESP_OK;
}

/** Configure PCA9685 to not respond to subaddress
 *  @param handle device handle
 *  @returns success or failure 
 */
esp_err_t pca9685_disable_sub1(pca9685_handle_t handle) {
    return clear_bits(handle, MODE1, SUB1);
}

/** Configure PCA9685 to respond to specified subaddress
 *  @param handle device handle
 *  @param all_call_addr i2c address to respond to
 *  @returns success or failure 
 */
esp_err_t pca9685_enable_sub2(pca9685_handle_t handle, uint8_t sub2_addr) {
    esp_err_t err __attribute__((unused));

    err = set_bits(handle, MODE1, SUB2);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to set bits");

    uint8_t buffer[2] = {SUBADR2, sub2_addr};
    err = i2c_master_transmit(handle->i2c_handle, buffer, 2, DEFAULT_TIMEOUT);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to write address");

    return ESP_OK;
}

/** Configure PCA9685 to not respond to subaddress
 *  @param handle device handle
 *  @returns success or failure 
 */
esp_err_t pca9685_disable_sub2(pca9685_handle_t handle) {
    return clear_bits(handle, MODE1, SUB2);
}

/** Configure PCA9685 to respond to specified subaddress
 *  @param handle device handle
 *  @param all_call_addr i2c address to respond to
 *  @returns success or failure 
 */
esp_err_t pca9685_enable_sub3(pca9685_handle_t handle, uint8_t sub3_addr) {
    esp_err_t err __attribute__((unused));

    err = set_bits(handle, MODE1, SUB3);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to set bits");

    uint8_t buffer[2] = {SUBADR3, sub3_addr};
    err = i2c_master_transmit(handle->i2c_handle, buffer, 2, DEFAULT_TIMEOUT);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to write address");

    return ESP_OK;
}

/** Configure PCA9685 to not respond to subaddress
 *  @param handle device handle
 *  @returns success or failure 
 */
esp_err_t pca9685_disable_sub3(pca9685_handle_t handle) {
    return clear_bits(handle, MODE1, SUB3);
}

/** Configure whether device outputs inverted
 *  @param handle device handle
 *  @param invrt invert mode 
 *  @returns success or failure 
 */
esp_err_t pca9685_config_inverted(pca9685_handle_t handle, PCA9685_OUTPUT_INVRT_MODE invrt) {
    switch (invrt) {
        case PCA9685_OUTPUT_INVERTED:
            return set_bits(handle, MODE2, INVRT);
        case PCA9685_OUTPUT_NORMAL:
            return clear_bits(handle, MODE2, INVRT);
    }
    return ESP_OK;
}

/** Configure when outputs change on i2c transmission
 *  @param handle device handle
 *  @param och output change mode
 *  @returns success or failure 
 */
esp_err_t pca9685_config_output_change(pca9685_handle_t handle, PCA9685_OUTPUT_CHANGE_MODE och) {
    switch (och) {
        case PCA9685_OUTPUT_CHANGE_ON_STOP:
            return clear_bits(handle, MODE2, OCH);
        case PCA9685_OUTPUT_CHANGE_ON_ACK:
            return set_bits(handle, MODE2, OCH);
    }
    return ESP_OK;
}

/** Configure output drive mode
 *  @param handle device handle
 *  @param drive_mode output drive mode
 *  @returns success or failure 
 */
esp_err_t pca9685_config_drive_mode(pca9685_handle_t handle, PCA9685_OUTPUT_DRIVE_MODE drive_mode) {
    switch (drive_mode) {
        case PCA9685_DRIVE_OPEN_DRAIN:
            return clear_bits(handle, MODE2, OUTDRV);
        case PCA9685_DRIVE_TOTEM_POLE:
            return set_bits(handle, MODE2, OUTDRV);
    }
    return ESP_OK;
}

/** Configure output disable mode
 *  @param handle device handle
 *  @param outne output disabled mode
 *  @returns success or failure 
 */
esp_err_t pca9685_config_output_disabled_mode(pca9685_handle_t handle, PCA9685_OUTPUT_NOT_ENABLED_MODE outne_mode) {
    esp_err_t err __attribute__((unused));
 
    switch (outne_mode) {
        case PCA9685_DISABLED_LOW: // MODE2[0:1] = 00
            return clear_bits(handle, MODE2, OUTNE_L | OUTNE_H);
        case PCA9685_DISABLED_HIGH: // MODE2[0:1] = 01
            err = clear_bits(handle, MODE2, OUTNE_H);
            ESP_RETURN_ON_ERROR(err, TAG, "failed to set bits");
            return set_bits(handle, MODE2, OUTNE_L);
        case PCA9685_DISABLED_HIGH_IMPEDENCE: // MODE2[0:1] = 1X
            return set_bits(handle, MODE2, OUTNE_H);
    }
    return ESP_OK;
}