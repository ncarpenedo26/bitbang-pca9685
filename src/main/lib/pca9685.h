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
esp_err_t set_channel_on(pca9685_handle_t handle, uint32_t channel); 
esp_err_t set_channel_off(pca9685_handle_t handle, uint32_t channel);

esp_err_t set_all(pca9685_handle_t handle, uint16_t on_counts, uint16_t off_counts);
esp_err_t set_all_duty_cycle(pca9685_handle_t handle, double duty_cycle, double phase_delay);
esp_err_t set_all_pulse_width(pca9685_handle_t handle, uint32_t pulse_width_us, uint32_t phase_shift_us);
esp_err_t set_all_on(pca9685_handle_t handle); 
esp_err_t set_all_off(pca9685_handle_t handle);


esp_err_t pca9685_sleep(pca9685_handle_t handle);
esp_err_t pca9685_wake(pca9685_handle_t handle);
esp_err_t pca9685_restart(pca9685_handle_t handle);
esp_err_t pca9685_use_extclk(pca9685_handle_t handle);
esp_err_t pca9685_set_prescale(pca9685_handle_t handle, uint8_t prescale);
esp_err_t pca9685_set_freq(pca9685_handle_t handle, uint32_t freq);

esp_err_t pca9685_set_inverted(pca9685_handle_t handle);
esp_err_t pca9685_set_not_inverted(pca9685_handle_t handle);
esp_err_t pca9685_config_open_drain(pca9685_handle_t handle);
esp_err_t pca9685_config_totem_pole(pca9685_handle_t handle);

esp_err_t pca9685_enable_all_call(pca9685_handle_t handle, uint8_t all_call_addr);
esp_err_t pca9685_disable_all_call(pca9685_handle_t handle);
esp_err_t pca9685_enable_sub1(pca9685_handle_t handle, uint8_t sub1_addr);
esp_err_t pca9685_disable_sub1(pca9685_handle_t handle);
esp_err_t pca9685_enable_sub2(pca9685_handle_t handle, uint8_t sub2_addr);
esp_err_t pca9685_disable_sub2(pca9685_handle_t handle);
esp_err_t pca9685_enable_sub3(pca9685_handle_t handle, uint8_t sub3_addr);
esp_err_t pca9685_disable_sub3(pca9685_handle_t handle);

#endif