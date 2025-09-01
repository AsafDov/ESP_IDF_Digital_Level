/*
    OLED LED DRIVER - Project by Asaf "Arliden The Bard" Dov
    Running random walk example. 
    adapt to your liking.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "oled_driver.h"
#include "mpu9250_driver.h"
#include "sdkconfig.h"

// I2c CONFIG
#define SCL_SPEED_HZ 400000
#define SCL_PIN GPIO_NUM_22
#define SDA_PIN GPIO_NUM_21

// OLED CONFIG
#define OLED_I2C_ADDRESS 0x3c
#define OLED_SCREEN_WIDTH 128
#define OLED_SCREEN_HEIGHT 64
#define OLED_STATUS_BAR_HEIGHT 16
#define OLED_CONTENT_HEIGHT (OLED_SCREEN_HEIGHT - OLED_STATUS_BAR_HEIGHT)
#define OLED_REFRESH_RATE_MS 250

// MPU9250 CONFIG
#define MPU9250_SENSOR_ADDR 0x68
// Globals:
typedef struct{
    oled_handle_t oled_handle;
    mpu9250_handle_t mpu9250_handle;
} task_params_t;

SemaphoreHandle_t i2c_bus_mutex; // Manages bus access

// Declarations
void i2c_master_install(i2c_master_bus_handle_t * pBus_handle);
void oled_install(i2c_master_bus_handle_t bus_handle, oled_handle_t *oled_handle);
void mpu9250_install(i2c_master_bus_handle_t bus_handle, mpu9250_handle_t *mpu9250_handle);
void level_task(void* params);

// Main
void app_main(void)
{
    static i2c_master_bus_handle_t bus_handle;
    static oled_handle_t oled_handle;
    static mpu9250_handle_t mpu9250_handle;
    
    /* Installing drivers  */
    i2c_master_install(&bus_handle);
    oled_install(bus_handle, &oled_handle);
    vTaskDelay(pdMS_TO_TICKS(1000));
    mpu9250_install(bus_handle, &mpu9250_handle); 
    vTaskDelay(pdMS_TO_TICKS(1000));

    static task_params_t params;
    
    params.oled_handle = oled_handle;
    params.mpu9250_handle = mpu9250_handle;


    /* Start LEVEL task example */
    xTaskCreatePinnedToCore(level_task, "level_task", 4096, &params, 5, NULL, 1);
}


/* Functions */
void i2c_master_install(i2c_master_bus_handle_t * pBus_handle)
{
    ESP_LOGI("I2C", "Configuring I2C Master");
    /* Configure and Initialize I2C Master */
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 0,
        .scl_io_num = SCL_PIN, 
        .sda_io_num = SDA_PIN, 
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    // Create the mutex
    i2c_bus_mutex = xSemaphoreCreateMutex();
    if (i2c_bus_mutex == NULL) {
        ESP_LOGE("I2C", "Failed to create I2C mutex");
    }
    xSemaphoreGive(i2c_bus_mutex);
    ESP_LOGI("I2C", "Initializing I2C Master");
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, pBus_handle));
}

void oled_install(i2c_master_bus_handle_t bus_handle, oled_handle_t *oled_handle){
    /* Configure and Instantiate OLED device */
    ESP_LOGI("OLED", "Configuring OLED");
    oled_config_t oled_cfg = {
        .device_address = OLED_I2C_ADDRESS,
        .address_bit_length = I2C_ADDR_BIT_LEN_7,
        .width = OLED_SCREEN_WIDTH,
        .height = OLED_SCREEN_HEIGHT,
        .refresh_rate_ms = OLED_REFRESH_RATE_MS,
        .scl_speed_hz = SCL_SPEED_HZ,
        .i2c_mutex = i2c_bus_mutex,
    };
    ESP_LOGI("OLED", "Initializing OLED");
    *oled_handle = oled_init(bus_handle, &oled_cfg);
    ESP_LOGI("OLED", "OLED Initialized");
}

void mpu9250_install(i2c_master_bus_handle_t bus_handle, mpu9250_handle_t *mpu9250_handle){
    static const char *TAG = "MPU 9250";
    /* Configure and Instantiate OLED device */
    ESP_LOGI(TAG, "Configuring MPU 9250");
    mpu9250_config_t mpu9250_cfg = {
        .device_address = MPU9250_SENSOR_ADDR,
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz = SCL_SPEED_HZ,
        .task_priority = 6,
        .i2c_mutex = i2c_bus_mutex
    };
    ESP_LOGI(TAG, "Initializing MPU");
    *mpu9250_handle = mpu9250_init(bus_handle, &mpu9250_cfg);
    ESP_LOGI(TAG, "MPU Initialized");
}

void level_task(void* params){
    task_params_t* task_params = (task_params_t*)params;
    oled_handle_t oled_handle = task_params->oled_handle;
    mpu9250_handle_t mpu9250_handle = task_params->mpu9250_handle;
    // queue_handle_t mpu9250_data_queue = mpu9250_get_data_queue(mpu9250_handle);
    quaternion_t* pQ;
    quaternion_t q;

    int center_x = OLED_SCREEN_WIDTH / 2; // TODO Implement get width and get height functions in the oled driver
    int center_y = OLED_CONTENT_HEIGHT/2 + OLED_STATUS_BAR_HEIGHT; //First 16 is the status bar
    int x = center_x, y = center_y, r = 0;
    float pitch, roll, yaw;
    while(1){   
        /* Orientation is defined as followed in my case:
        *   - pitch: rotation around the Z-axis
        *   - roll: rotation around the X-axis
        *   - yaw: rotation around the Y-axis
        */
        pQ = mpu9250_get_quaternion(mpu9250_handle);
        q = *pQ;
        quaternion_to_euler(q, &pitch, &roll, &yaw);
        r = 5;
        x = (int)(center_x - pitch);
        y = (int)(center_y - (roll)); // 95 is the orientation offset on my mpu
        if (x < r) x = r;
        if (y < r+OLED_STATUS_BAR_HEIGHT) y = r+OLED_STATUS_BAR_HEIGHT;
        if (x > OLED_SCREEN_WIDTH - 1 - r) x = OLED_SCREEN_WIDTH - 1 - r;
        if (y > OLED_SCREEN_HEIGHT - 1 - r) y = OLED_SCREEN_HEIGHT - 1 - r;

        // ESP_LOGI("LEVEL", "pitch: %f, roll: %f, yaw: %f", pitch, roll, yaw);
        // ESP_LOGI("LEVEL", "x: %d, y: %d", x, y);

        /* Clear screen */
        oled_clean_screen(oled_handle);

        /* Draw cross */
        oled_square_filled(oled_handle, 0, center_y, 127, center_y);
        oled_square_filled(oled_handle, center_x, 16, center_x, 63);

        /* Draw circle */
        oled_circle_filled(oled_handle, x, y, r);

        /* Update OLED */
        oled_update_frame_buffer(oled_handle);

        // No need for delay due to the wait on the i2c bus
    }

}
