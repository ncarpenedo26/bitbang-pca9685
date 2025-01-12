#ifndef PCA9685_H
#define PCA9685_H

#include "driver/i2c_slave.h"
#include "driver/i2c_master.h"

typedef struct pca9685_dev {
    i2c_master_dev_handle_t i2c_handle;
} pca9685_dev_t;

typedef pca9685_dev_t* pca9685_handle_t;

esp_err_t setup_pca9685(i2c_master_bus_handle_t bus_handle, const i2c_device_config_t *dev_config, pca9685_handle_t *ret_handle);
esp_err_t set_channel(pca9685_handle_t handle, uint32_t channel, uint16_t on_counts, uint16_t off_counts);
esp_err_t set_channel_duty_cycle(pca9685_handle_t handle, uint32_t channel, double duty_cycle, double phase_delay);
esp_err_t set_channel_pulse_width(pca9685_handle_t handle, uint32_t channel, uint32_t pulse_width_us, uint32_t phase_shift_us);

#endif