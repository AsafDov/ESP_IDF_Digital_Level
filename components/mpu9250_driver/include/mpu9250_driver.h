#pragma once
/**
 * MPU9250 driver for ESP IDF
 * Created by Asaf "Arliden The Bard"
 */

#include <stdio.h>
#include "driver/i2c_master.h"

typedef struct mpu *mpu9250_handle_t;

typedef struct mpu_config{
    i2c_addr_bit_len_t dev_addr_length;         /*!< Select the address length of the slave device. */
    uint16_t device_address;                    /*!< I2C device raw address. (The 7/10 bit address without read/write bit). Macro I2C_DEVICE_ADDRESS_NOT_USED (0xFFFF) stands for skip the address config inside driver. */
    uint32_t scl_speed_hz;                      /*!< I2C SCL line frequency. */
    uint32_t scl_wait_us;                      /*!< Timeout value. (unit: us). Please note this value should not be so small that it can handle stretch/disturbance properly. If 0 is set, that means use the default reg value*/
    uint8_t  task_priority;                     /*!< Task priority for the I2C communication task. */
    SemaphoreHandle_t i2c_mutex;                /*!< I2C mutex for synchronization. */
    struct {
        uint32_t disable_ack_check:      1;     /*!< Disable ACK check. If this is set false, that means ack check is enabled, the transaction will be stopped and API returns error when nack is detected. */
    } flags;                                    /*!< I2C device config flags */
} mpu9250_config_t;

typedef struct {
    float w;
    float x;
    float y;
    float z;
} quaternion_t;

typedef QueueHandle_t queue_handle_t;

mpu9250_handle_t mpu9250_init(i2c_master_bus_handle_t bus_handle, mpu9250_config_t* mpu9250_cfg);
queue_handle_t mpu9250_get_data_queue(mpu9250_handle_t mpu_handle);
void quaternion_to_euler(quaternion_t q, float* pitch, float* roll, float* yaw);
quaternion_t* mpu9250_get_quaternion(mpu9250_handle_t handle);
