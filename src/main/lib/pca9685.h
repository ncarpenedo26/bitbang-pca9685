#ifndef PCA9685_H
#define PCA9685_H

#include "driver/i2c_slave.h"
#include "driver/i2c_master.h"

#define PCA9685_DEFAULT_I2C_ADDR      0x40
#define PCA9685_DEFAULT_ALLCALLADR    0xE0
#define PCA9685_DEFAULT_SUB1ADR       0xE2
#define PCA9685_DEFAULT_SUB2ADR       0xE4
#define PCA9685_DEFAULT_SUB3ADR       0xE8

// Device State
typedef struct pca9685_dev {
    i2c_master_dev_handle_t i2c_handle;
} pca9685_dev_t;

typedef pca9685_dev_t* pca9685_handle_t;

// Invert output logic state
typedef enum {
    PCA9685_OUTPUT_NORMAL = 0,
    PCA9685_OUTPUT_INVERTED = 1,
} PCA9685_OUTPUT_INVRT_MODE;

// Output drive mode
typedef enum {
    PCA9685_DRIVE_OPEN_DRAIN = 0,
    PCA9685_DRIVE_TOTEM_POLE = 1,
} PCA9685_OUTPUT_DRIVE_MODE;

// Set when outputs change from i2c transmission
typedef enum {
    PCA9685_OUTPUT_CHANGE_ON_STOP = 0,
    PCA9685_OUTPUT_CHANGE_ON_ACK = 1,
} PCA9685_OUTPUT_CHANGE_MODE;

// Output state when !OE pin HIGH
typedef enum {
    PCA9685_DISABLED_LOW = 0,
    PCA9685_DISABLED_HIGH = 1,
    PCA9685_DISABLED_HIGH_IMPEDENCE = 2,
} PCA9685_OUTPUT_NOT_ENABLED_MODE;

esp_err_t pca9685_init(i2c_master_bus_handle_t bus_handle, const i2c_device_config_t *dev_config, pca9685_handle_t *ret_handle);

esp_err_t pca9685_get_prescale(pca9685_handle_t handle, uint8_t* prescale);
esp_err_t pca9685_set_prescale(pca9685_handle_t handle, uint8_t prescale);
esp_err_t pca9685_get_freq(pca9685_handle_t handle, uint32_t* freq);
esp_err_t pca9685_set_freq(pca9685_handle_t handle, uint32_t freq);

esp_err_t pca9685_set_duty_cycle(pca9685_handle_t handle, uint32_t channel, double duty_cycle, double phase_delay);
esp_err_t pca9685_set_pulse_width(pca9685_handle_t handle, uint32_t channel, uint32_t pulse_width_us, uint32_t phase_shift_us);
esp_err_t pca9685_set_channel_on(pca9685_handle_t handle, uint32_t channel);
esp_err_t pca9685_set_channel_off(pca9685_handle_t handle, uint32_t channel);

esp_err_t pca9685_set_all_on(pca9685_handle_t handle);
esp_err_t pca9685_set_all_off(pca9685_handle_t handle);
esp_err_t pca9685_set_all_duty_cycle(pca9685_handle_t handle, double duty_cycle, double phase_delay);
esp_err_t pca9685_set_all_pulse_width(pca9685_handle_t handle, uint32_t pulse_width_us, uint32_t phase_shift_us);

esp_err_t pca9685_disable(pca9685_handle_t handle);
esp_err_t pca9685_enable(pca9685_handle_t handle);
esp_err_t pca9685_restart(pca9685_handle_t handle);
esp_err_t pca9685_use_extclk(pca9685_handle_t handle);

esp_err_t pca9685_enable_all_call(pca9685_handle_t handle, uint8_t all_call_addr);
esp_err_t pca9685_disable_all_call(pca9685_handle_t handle);
esp_err_t pca9685_enable_sub1(pca9685_handle_t handle, uint8_t sub1_addr);
esp_err_t pca9685_disable_sub1(pca9685_handle_t handle);
esp_err_t pca9685_enable_sub2(pca9685_handle_t handle, uint8_t sub2_addr);
esp_err_t pca9685_disable_sub2(pca9685_handle_t handle);
esp_err_t pca9685_enable_sub3(pca9685_handle_t handle, uint8_t sub3_addr);
esp_err_t pca9685_disable_sub3(pca9685_handle_t handle);

esp_err_t pca9685_config_inverted(pca9685_handle_t handle, PCA9685_OUTPUT_INVRT_MODE invrt);
esp_err_t pca9685_config_output_change(pca9685_handle_t handle, PCA9685_OUTPUT_CHANGE_MODE och);
esp_err_t pca9685_config_drive_mode(pca9685_handle_t handle, PCA9685_OUTPUT_DRIVE_MODE drive_mode);
esp_err_t pca9685_config_output_disabled_mode(pca9685_handle_t handle, PCA9685_OUTPUT_NOT_ENABLED_MODE outne_mode);

#endif