#include <math.h>
#include "driver/i2c.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "mpu9250_driver.h"
#include "mpu9240_register_map.h"

#define TAG "MPU9250"
#define I2C_MASTER_TIMEOUT_MS 1000

// Declarations
void mpu9250_send_command(mpu9250_handle_t mpu9250_handle, unsigned char* cmd, int cmd_size);
void configure_magnetometer(mpu9250_handle_t mpu_handle);
void check_buffer_task(void* mpu9250_handle);

typedef struct{
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    float dt;
    float gx_bias, gy_bias, gz_bias;
    float mx_bias, my_bias, mz_bias;
}angles_buf_t;

struct mpu{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t i2c_device_mpu9250_handle; 
    i2c_master_dev_handle_t i2c_device_mag_handle; 
    i2c_addr_bit_len_t dev_addr_length;         // Select the address length of the slave device.
    uint16_t device_address;                    // I2C device raw address. (The 7/10 bit address without read/write bit).
    uint32_t scl_speed_hz;                      // I2C SCL line frequency.
    uint32_t scl_wait_us;                       // Timeout value in us.
    SemaphoreHandle_t i2c_mutex;                // I2C mutex for synchronization.
    uint16_t accel_sensitivity;                 // Accelerometer sensitivity setting.
    uint8_t  task_priority;
    angles_buf_t angles_buf;
    quaternion_t q;
    float integral_error[3];                    // For mahony filter, might change to magewick eventually
    uint64_t last_update_time_us;
    queue_handle_t data_queue;
    SemaphoreHandle_t data_semaphore;
};

void apply_mahony_filter(mpu9250_handle_t handle){
    float q0 = handle->q.w, q1 = handle->q.x, q2 = handle->q.y, q3 = handle->q.z;
    float ax = handle->angles_buf.ax, ay = handle->angles_buf.ay, az = handle->angles_buf.az;
    float mx = handle->angles_buf.mx, my = handle->angles_buf.my, mz = handle->angles_buf.mz;
    float gx = handle->angles_buf.gx, gy = handle->angles_buf.gy, gz = handle->angles_buf.gz;
    float norm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    
    /* Get the time delta for the integral */ 
    uint64_t current_time_us = esp_timer_get_time();
    handle->angles_buf.dt = (handle->last_update_time_us == 0) ? 0.01f : (float)(current_time_us - handle->last_update_time_us) / 1000000.0f;
    handle->last_update_time_us = current_time_us;
    float dt = handle->angles_buf.dt;

    /* --- Tuning parameters for the Mahony filter --- */
    const float Kp = 2.0f; // Proportional gain
    const float Ki = 0.10f; // Integral gain

    /* Normalize accelerometer measurement */
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm > 0.0f) {
        ax /= norm;
        ay /= norm;
        az /= norm;
    }

    /* Normalize magnetometer measurement */
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm > 0.0f) {
        mx /= norm;
        my /= norm;
        mz /= norm;
    }

    /* Reference direction of Earth's magnetic field */ 
    hx = 2.0f * (mx * (0.5f - q2 * q2 - q3 * q3) + my * (q1 * q2 - q0 * q3) + mz * (q1 * q3 + q0 * q2));
    hy = 2.0f * (mx * (q1 * q2 + q0 * q3) + my * (0.5f - q1 * q1 - q3 * q3) + mz * (q2 * q3 - q0 * q1));
    bx = sqrtf(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1 * q3 - q0 * q2) + my * (q2 * q3 + q0 * q1) + mz * (0.5f - q1 * q1 - q2 * q2));

    /* Estimated direction of gravity and magnetic field */ 
    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    wx = 2.0f * (bx * (0.5f - q2 * q2 - q3 * q3) + bz * (q1 * q3 - q0 * q2));
    wy = 2.0f * (bx * (q1 * q2 - q0 * q3) + bz * (q0 * q1 + q2 * q3));
    wz = 2.0f * (bx * (q0 * q2 + q1 * q3) + bz * (0.5f - q1 * q1 - q2 * q2));

    /* Error is cross product between estimated and measured direction */ 
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

    /* Integrate error terms */
    handle->integral_error[0] += ex * Ki * dt;
    handle->integral_error[1] += ey * Ki * dt;
    handle->integral_error[2] += ez * Ki * dt;
    
    /* Apply feedback */ 
    gx += Kp * ex + handle->integral_error[0];
    gy += Kp * ey + handle->integral_error[1];
    gz += Kp * ez + handle->integral_error[2];

    /* Integrate rate of change of quaternion */ 
    q0 += (-q1 * gx - q2 * gy - q3 * gz) * 0.5f * dt;
    q1 += (q0 * gx + q2 * gz - q3 * gy) * 0.5f * dt;
    q2 += (q0 * gy - q1 * gz + q3 * gx) * 0.5f * dt;
    q3 += (q0 * gz + q1 * gy - q2 * gx) * 0.5f * dt;

    /* Normalize quaternion and update handle */
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    handle->q.w = q0 / norm;
    handle->q.x = q1 / norm;
    handle->q.y = q2 / norm;
    handle->q.z = q3 / norm;
}

void read_magnetometer_data(mpu9250_handle_t handle, uint8_t* mag_data){

    uint8_t mag_reg_addr = MPU9250_MAG_HXL_REG_ADDR;
    uint8_t bypass_cmd[] = {MPU9250_INT_PIN_CFG_REG_ADDR, 0x00};

    /* Enable Bypass Mode */
    bypass_cmd[1] = 0x02; // Set BYPASS_EN bit
    i2c_master_transmit(handle->i2c_device_mpu9250_handle, bypass_cmd, 2, -1);
    
    /* Read Magnetometer directly */
    i2c_master_transmit_receive(handle->i2c_device_mag_handle, &mag_reg_addr, 1, mag_data, 7, -1);
    
    /* Disable Bypass Mode */
    bypass_cmd[1] = 0x00; // Clear BYPASS_EN bit
    i2c_master_transmit(handle->i2c_device_mpu9250_handle, bypass_cmd, 2, -1);

}

// Not using FIFO buffer
void mpu9250_task(void* mpu9250_handle){

    /* SETUP */
    mpu9250_handle_t handle = (mpu9250_handle_t)mpu9250_handle;
    vTaskDelay(pdMS_TO_TICKS(100));
    uint8_t write_buffer = MPU9250_ACCEL_XOUT_H_REG_ADDR;
    uint8_t mag_data[7]; // Buffer and register for direct magnetometer read
    uint8_t raw_data_buffer[14];

    /* LOOP */
    while (1) {
        if (xSemaphoreTake(handle->data_semaphore, portMAX_DELAY) == pdTRUE){
            
            if (xSemaphoreTake(handle->i2c_mutex, portMAX_DELAY) == pdTRUE){
                /* GET SENSOR DATA FROM REGISTERS! LET GO OF THE FIFO DAMNIT */
                write_buffer = MPU9250_ACCEL_XOUT_H_REG_ADDR;
                ESP_ERROR_CHECK(i2c_master_transmit_receive(handle->i2c_device_mpu9250_handle, &write_buffer, 1, raw_data_buffer, sizeof(raw_data_buffer), -1));
                read_magnetometer_data(handle, mag_data);
                xSemaphoreGive(handle->i2c_mutex);
                int16_t accel_x_raw = (int16_t)((raw_data_buffer[0] << 8)  | raw_data_buffer[1]);
                int16_t accel_y_raw = (int16_t)((raw_data_buffer[2] << 8)  | raw_data_buffer[3]);
                int16_t accel_z_raw = (int16_t)((raw_data_buffer[4] << 8)  | raw_data_buffer[5]);
                
                // 6 and 7 are temperature
                
                int16_t gyro_x_raw = (int16_t)((raw_data_buffer[8] << 8)  | raw_data_buffer[9]);
                int16_t gyro_y_raw = (int16_t)((raw_data_buffer[10] << 8) | raw_data_buffer[11]);
                int16_t gyro_z_raw = (int16_t)((raw_data_buffer[12] << 8) | raw_data_buffer[13]);
                
                handle->accel_sensitivity = MPU9250_ACCEL_SENSITIVITY_4G;
                
                handle->angles_buf.ax = (float)accel_x_raw / handle->accel_sensitivity; // (in g's)
                handle->angles_buf.ay = (float)accel_y_raw / handle->accel_sensitivity; // (in g's)
                handle->angles_buf.az = (float)accel_z_raw / handle->accel_sensitivity; // (in g's)
                                
                handle->angles_buf.gx = ((float)gyro_x_raw / MPU9250_GYRO_SENSITIVITY_500dps) * M_PI / 180.0f; // In degrees per second
                handle->angles_buf.gy = ((float)gyro_y_raw / MPU9250_GYRO_SENSITIVITY_500dps) * M_PI / 180.0f;
                handle->angles_buf.gz = ((float)gyro_z_raw / MPU9250_GYRO_SENSITIVITY_500dps) * M_PI / 180.0f;

                int16_t mag_x_raw = (int16_t)((mag_data[1] << 8) | mag_data[0]);
                int16_t mag_y_raw = (int16_t)((mag_data[3] << 8) | mag_data[2]);
                int16_t mag_z_raw = (int16_t)((mag_data[5] << 8) | mag_data[4]);
                
                handle->angles_buf.mx = (float)mag_y_raw * 0.6;
                handle->angles_buf.my = (float)mag_x_raw * 0.6;
                handle->angles_buf.mz = -(float)mag_z_raw * 0.6;
                
                handle->angles_buf.gx -= handle->angles_buf.gx_bias;
                handle->angles_buf.gy -= handle->angles_buf.gy_bias;
                handle->angles_buf.gz -= handle->angles_buf.gz_bias;
                
                apply_mahony_filter(handle);
            }
        }
    }
}

/* This function retreives quaternion adhoc */
quaternion_t* mpu9250_get_quaternion(mpu9250_handle_t handle){
    xSemaphoreGive(handle->data_semaphore);
    return &handle->q;
}

queue_handle_t mpu9250_get_data_queue(mpu9250_handle_t mpu_handle){
    if (mpu_handle) {
        return mpu_handle->data_queue;
    }
    return NULL;
}

void mpu9250_send_command(mpu9250_handle_t mpu9250_handle, unsigned char* cmd, int cmd_size){
    ESP_ERROR_CHECK(i2c_master_transmit(mpu9250_handle->i2c_device_mpu9250_handle, cmd, cmd_size, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
    vTaskDelay(pdMS_TO_TICKS(20));
}

void configure_magnetometer(mpu9250_handle_t mpu_handle){
    uint8_t write_buf[2] = {0x00};
  
    /* Configure Magnetometer */
    ESP_LOGI(TAG, "Configuring Magnetometer");

    ESP_LOGI(TAG, "Disabling I2C master interface");
    /* Disable I2C Master and enable Bypass Mode */
    write_buf[0] = MPU9250_USER_CTRL_REG_ADDR;
    write_buf[1] = 0x00; // Disable I2C Master
    i2c_master_transmit(mpu_handle->i2c_device_mpu9250_handle, write_buf, 2, -1);

    /* Enable Bypass */
    ESP_LOGI(TAG, "Enabling Bypass");
    write_buf[0] = MPU9250_INT_PIN_CFG_REG_ADDR;
    write_buf[1] = 0x02;
    mpu9250_send_command(mpu_handle, write_buf, sizeof(write_buf));
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Checking Who Am I register */ 
    uint8_t wia_reg = MPU9250_MAG_WIA_REG_ADDR; // WIA register is at 0x00
    uint8_t wia_val = 0;
    esp_err_t err = i2c_master_transmit_receive(mpu_handle->i2c_device_mag_handle, &wia_reg, 1, &wia_val, 1, pdMS_TO_TICKS(100));
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "AK8963 WIA (Who Am I): 0x%02X (should be 0x48)", wia_val);
        if (wia_val != 0x48) {
            ESP_LOGE(TAG, "Invalid WIA value for AK8963. Check I2C connections and pull-up resistors.");
        }
    } else {
        ESP_LOGE(TAG, "Failed to read AK8963 WIA register. Magnetometer not responding.");
    }

    /* ROBUST INIT: Power Down magnetometer first --- */
    ESP_LOGI(TAG, "Powering down magnetometer to ensure known state.");
    uint8_t mag_cmd[2];
    mag_cmd[0] = MPU9250_MAG_CNTL1_REG_ADDR;
    mag_cmd[1] = 0x00; // Power-down mode
    i2c_master_transmit(mpu_handle->i2c_device_mag_handle, mag_cmd, 2, pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Set MODE of Magnetometer */
    mag_cmd[0] = MPU9250_MAG_CNTL1_REG_ADDR;
    mag_cmd[1] = MPU9250_MAG_MODE_CONTINUOUS_100HZ;
    err = i2c_master_transmit(mpu_handle->i2c_device_mag_handle, mag_cmd, 2, 1000 / portTICK_PERIOD_MS);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Successfully sent command to magnetometer!");
    } else {
        ESP_LOGE(TAG, "Failed to communicate with magnetometer.");
    }

    /* Disable Bypass */
    write_buf[0] = MPU9250_INT_PIN_CFG_REG_ADDR;
    write_buf[1] = 0x00;
    mpu9250_send_command(mpu_handle, write_buf, sizeof(write_buf));
    vTaskDelay(pdMS_TO_TICKS(50));
}

void send_configuration_commands(mpu9250_handle_t mpu_handle){
    ESP_LOGI(TAG, "Starting MPU9250 configuration...");
     if (xSemaphoreTake(mpu_handle->i2c_mutex, portMAX_DELAY) == pdTRUE) {
        uint8_t who_am_i_reg = MPU9250_WHO_AM_I_REG_ADDR; // This is register 0x75
        uint8_t device_id = 0;
        i2c_master_transmit_receive(mpu_handle->i2c_device_mpu9250_handle, &who_am_i_reg, 1, &device_id, 1, -1);
        ESP_LOGI(TAG, "======================================================");
        ESP_LOGI(TAG, "MPU WHO_AM_I register is: 0x%02X", device_id);
        if (device_id == 0x71) {
            ESP_LOGI(TAG, "Device is a genuine MPU9250. The magnetometer might be faulty.");
        } else if (device_id == 0x70) {
            ESP_LOGE(TAG, "Device is an MPU6500. This is a 6-axis sensor with NO MAGNETOMETER.");
        } else if (device_id == 0x68) {
             ESP_LOGE(TAG, "Device is an MPU6050. This is a 6-axis sensor with NO MAGNETOMETER.");
        } else {
            ESP_LOGW(TAG, "Device ID is unrecognized. It is likely a clone without a magnetometer.");
        }
        ESP_LOGI(TAG, "======================================================");
    
        ESP_LOGI(TAG, "Sending configuration commands to MPU9250");
        // Send the necessary configuration commands to the MPU9250
        uint8_t write_buf[2] = {0x00};
        uint8_t data = 0x00;
        
        /* Wake up the MPU9250 */
        ESP_LOGI(TAG, "Waking up MPU9250");
        data = 0x80;
        write_buf[0] = MPU9250_PWR_MGMT_1_REG_ADDR;
        write_buf[1] = data;
        mpu9250_send_command(mpu_handle, write_buf, 2);
        vTaskDelay(pdMS_TO_TICKS(100));
    
        data = 0x01;
        write_buf[0] = MPU9250_PWR_MGMT_1_REG_ADDR;
        write_buf[1] = data;
        mpu9250_send_command(mpu_handle, write_buf, 2);
    
        /* Enable Gyroscope and Accelerometer */
        ESP_LOGI(TAG, "Enabling Gyroscope and Accelerometer");
        data = 0x00;
        write_buf[0] = MPU9250_PWR_MGMT_2_REG_ADDR;
        write_buf[1] = data;
        mpu9250_send_command(mpu_handle, write_buf, sizeof(write_buf));
    
        
        /* Configure Gyroscope */
        ESP_LOGI(TAG, "Configuring Gyroscope");
        data = 0x08; // Set gyro config to +-500 degrees/sec
        write_buf[0] = MPU9250_GYRO_CONFIG_REG_ADDR;
        write_buf[1] = data;
        mpu9250_send_command(mpu_handle, write_buf, sizeof(write_buf));
        
        /* Configure Accelerometer */
        ESP_LOGI(TAG, "Configuring Accelerometer");
        data = 0x08; // Set accel config to +-4g
        write_buf[0] = MPU9250_ACCEL_CONFIG_REG_ADDR;
        write_buf[1] = data;
        mpu9250_send_command(mpu_handle, write_buf, sizeof(write_buf));
        
        /* Configure DLPF for Gyro */
        //! Was 0x03 for replacing buffer info with new info.
        //! Setting to 0x43 to allow buffer full to not overwrite old values
        ESP_LOGI(TAG, "Configuring DLPF for Gyro");
        data = 0x43; // Set DLPF to 41Hz meaning 5.9ms delay between gyro reads. Higher value is higher delay
        write_buf[0] = MPU9250_CONFIG_REG_ADDR;
        write_buf[1] = data;
        mpu9250_send_command(mpu_handle, write_buf, sizeof(write_buf));
        
        /* Configure DLPF for Accel */
        ESP_LOGI(TAG, "Configuring DLPF for Accel");
        data = 0x03; // Set DLPF to 44.8Hz meaning 4.88ms delay between accel reads.
        write_buf[0] = MPU9250_ACCEL_CONFIG_2_REG_ADDR;
        write_buf[1] = data;
        mpu9250_send_command(mpu_handle, write_buf, sizeof(write_buf));
        
        /* Set Sample Rate for DLPF */
        ESP_LOGI(TAG, "Setting Sample Rate for DLPF");
        data = 0x09; // Set sample rate to 200Hz
        write_buf[0] = MPU9250_SMPLRT_DIV_REG_ADDR;
        write_buf[1] = data;
        mpu9250_send_command(mpu_handle, write_buf, sizeof(write_buf));
        
        configure_magnetometer(mpu_handle);
    
        /* Enable FIFO register and Reset sensor registers and signal paths */
        ESP_LOGI(TAG, "Resetting and Enabling FIFO");
        write_buf[0] = MPU9250_USER_CTRL_REG_ADDR;
        write_buf[1] = 0x40; //0x60 enables I2C Master Interface Mode as well 
        mpu9250_send_command(mpu_handle, write_buf, sizeof(write_buf));
        
        /* Configure FIFO Enable Register for accelerometer and gyro and Magnetometer*/ 
        ESP_LOGI(TAG, "Configuring FIFO Enable Register");
        data = 0x78; // Enable FIFO for Gyro and Accelerometer. Disable slv, and temp output 0x79 to enable slave0 (Magnetometer) into FIFO
        write_buf[0] = MPU9250_FIFO_EN_REG_ADDR;
        write_buf[1] = data;
        mpu9250_send_command(mpu_handle, write_buf, sizeof(write_buf));
        
        xSemaphoreGive(mpu_handle->i2c_mutex);
     }
}

void quaternion_to_euler(quaternion_t q, float* pitch, float* roll, float* yaw) {
    /* roll (x-axis rotation) */
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    *roll = atan2f(sinr_cosp, cosr_cosp);

    /* pitch (z-axis rotation) */
    float sinp = 2 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        *pitch = copysignf(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        *pitch = asinf(sinp);

    // yaw (y-axis rotation)
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    *yaw = atan2f(siny_cosp, cosy_cosp);
    
    // Convert to degrees
    *pitch *= 180.0f / M_PI;
    *roll *= 180.0f / M_PI;
    *yaw *= 180.0f / M_PI;
}

mpu9250_handle_t mpu9250_init(i2c_master_bus_handle_t bus_handle, mpu9250_config_t* mpu9250_cfg){
    // Allocate memory for the main driver structure
    mpu9250_handle_t handle = (mpu9250_handle_t)malloc(sizeof(struct mpu));
    if (handle == NULL) {
        ESP_LOGI(TAG, "MPU9250 handle malloc failed");
        return NULL;
    }

    handle->bus_handle = bus_handle;
    handle->device_address = mpu9250_cfg->device_address;
    handle->dev_addr_length = mpu9250_cfg->dev_addr_length;
    handle->scl_speed_hz = mpu9250_cfg->scl_speed_hz;
    handle->scl_wait_us = mpu9250_cfg->scl_wait_us;
    handle->task_priority = mpu9250_cfg->task_priority;
    handle->angles_buf.dt = esp_timer_get_time();
    handle->i2c_mutex = mpu9250_cfg->i2c_mutex;

    /* Configure the I2C device settings */
    i2c_device_config_t i2c_device_mpu9250_cfg = {
        .dev_addr_length = handle->dev_addr_length,
        .device_address = handle->device_address,
        .scl_speed_hz = handle->scl_speed_hz,
    };
    
    /* Initialize the I2C device handle */
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &i2c_device_mpu9250_cfg, &handle->i2c_device_mpu9250_handle));
    ESP_LOGI(TAG, "MPU9250 device added to I2C bus");

    /* Create and store the I2C device handle for the Magnetometer */
    i2c_device_config_t mag_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU9250_MAG_DEVICE_ADDR, 
        .scl_speed_hz = handle->scl_speed_hz, 
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &mag_cfg, &handle->i2c_device_mag_handle));
    ESP_LOGI(TAG, "AK8963 Magnetometer device added to I2C bus");

    /* Initialize the orientation quaternion to identity */
    handle->q.w = 1.0f;
    handle->q.x = 0.0f;
    handle->q.y = 0.0f;
    handle->q.z = 0.0f;
    
    /* Initialize integral error for the fusion algorithm */
    handle->integral_error[0] = 0.0f;
    handle->integral_error[1] = 0.0f;
    handle->integral_error[2] = 0.0f;
    handle->angles_buf.gx_bias = -0.07f;
    handle->angles_buf.gy_bias = 0.04f;
    handle->angles_buf.gz_bias = -0.01f;

    handle->last_update_time_us = 0;

    /* initialize data queue */
    handle->data_queue = xQueueCreate(10, sizeof(quaternion_t)); 
    handle->data_semaphore = xSemaphoreCreateBinary();
    
    /* Configure the MPU9250 */
    send_configuration_commands(handle);

    /* Start Sensor Reading Task */
    xTaskCreatePinnedToCore(mpu9250_task, "MPU9250_task", 2048, handle, handle->task_priority, NULL, 1);

    return handle;
}