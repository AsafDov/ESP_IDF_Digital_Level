#include "driver/i2c.h"
#include <stdlib.h>
#include "esp_log.h"
#include <string.h>
#include "math.h"
/**
 * OLED driver for ESP IDF
 * Created by Asaf "Arliden The Bard" Dov
 */
 
#include "oled_driver.h"

#define I2C_MASTER_TIMEOUT_MS 100000
#define TAG "OLED"

const unsigned char clear_screen_bitmap[1024] = {0};

struct oled{
    i2c_master_bus_handle_t bus_handle;
    uint16_t device_address;
    i2c_addr_bit_len_t address_bit_length;
    int width;
    int height;
    int refresh_rate_ms;
    uint32_t scl_speed_hz;
    SemaphoreHandle_t i2c_mutex;
    i2c_master_dev_handle_t i2c_device_oled_handle;
    unsigned char* frame_buffer; 
};

int oled_get_refresh_rate(oled_handle_t oled_mpu_handle) {
    return oled_mpu_handle->refresh_rate_ms;
}

void send_command(oled_handle_t oled_handle,unsigned char* cmd, int cmd_size){
    esp_err_t ret = ESP_FAIL;
    switch (cmd_size){
        case 2:
        case 3:
        case 4:
            ret = i2c_master_transmit(oled_handle->i2c_device_oled_handle, cmd, cmd_size, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        break;

        default:
            ESP_LOGI("OLED", "Invalid array size in send_command");
    }

    if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "Bus is busy");
    } else if(ret == ESP_FAIL) {
        ESP_LOGW(TAG, "Write Failed");
    }
}

void reset_cursor(oled_handle_t oled_handle) {
    if (xSemaphoreTake(oled_handle->i2c_mutex, portMAX_DELAY) == pdTRUE) {
        // Set Column Address Range
        // This command requires 3 bytes: Command (0x21), Start Address (0x00), End Address (0x7F)
        unsigned char set_column_address[4];
        // Set Column Address Range
        set_column_address[0] = 0x00;
        set_column_address[1] = 0x21; // Command byte
        set_column_address[2] = 0x00; // Start address
        set_column_address[3] = 0x7F; // End address
        send_command(oled_handle, set_column_address, 4);
    
        // Set Page Address Range
        // This command requires 3 bytes: Command (0x22), Start Address (0x00), End Address (0x07)
        unsigned char set_page_address[4];
        // Set Page Address Range
        set_page_address[0] = 0x00;
        set_page_address[1] = 0x22; // Command byte
        set_page_address[2] = 0x00; // Start address
        set_page_address[3] = 0x07; // End address
        send_command(oled_handle, set_page_address, 4);

        xSemaphoreGive(oled_handle->i2c_mutex);
    }
}

// @brief Convert bitmap to page buffer format for OLED - UNUSED 
void bitmap_to_page_buffer(oled_handle_t oled_handle, unsigned char* bitmap, unsigned char* page_buffer, unsigned int page_buffer_size){
    // Page index starts from 1 because the first byte holds the command byte 0x40
    // Take each byte. And it with a Mask of 2^j
    unsigned char column;

    // for(int page=1; page<page_buffer_size; page++){
    //     for(int i=0; i<(oled_handle->width * oled_handle->height) - 8; i=i+8){
    //         for(int j=7; j>=0; j--){
    //             // Shift column left and OR with bitmap data
    //             column = (column << 1) | bitmap[i + j * oled_handle->width];
    //         }
    //     }
    //     page_buffer[page] = column;
    // }

    for(int page_num=0; page_num<8; page_num++){
        for (int byte_num=page_num*128; byte_num<(page_num+1)*128; byte_num++ ){
            for(int bit_num=256; bit_num>0; bit_num >>= 1){
                for(int k=0; k<8; k++){
                    column |= (bitmap[byte_num+k*16] & (bit_num>>1)) >> 1 ;
                }
                page_buffer[byte_num] = column;
            }
        }
    }

}


/**
 * @brief Clears the entire OLED display.
 * @details This function fills the display's internal Graphics RAM (GDDRAM)
 * with zeros, effectively turning off all pixels on the screen. It
 * allocates a temporary buffer, fills it with a pre-defined "clear"
 * bitmap, and transmits it to the display.
 * * @param oled_handle A handle to the initialized OLED driver instance.
 */
void oled_flush_gddram(oled_handle_t oled_handle) {
    int write_buf_size = (oled_handle->width * oled_handle->height) / 8 + 1;
    uint8_t* write_buf = (uint8_t*)malloc(write_buf_size);
    if (write_buf == NULL) {
        ESP_LOGE("Flush GDDRAM", "Failed to allocate memory for write buffer");
        return;
    }
    // The first byte of the transmission is the data command byte
    write_buf[0] = 0x40;
    
    // Prepare a bitmap of all zeros
    int bitmap_size = (oled_handle->width * oled_handle->height) / 8;
    memcpy(&write_buf[1], clear_screen_bitmap, bitmap_size);
    
    // Ensure the display's memory pointer is at the start (0,0)
    reset_cursor(oled_handle);

    // Send the clear buffer to the OLED display
    ESP_ERROR_CHECK(i2c_master_transmit(oled_handle->i2c_device_oled_handle, write_buf, write_buf_size, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));

    free(write_buf);
}

/**
 * @brief Initializes the SSD1306 controller with a standard configuration sequence.
 * @details This function sends a series of commands to the OLED display to configure
 * its operating parameters, such as display size, memory addressing mode,
 * contrast, and orientation. It follows a typical initialization flow recommended
 * for 128x64 displays. This is called once by oled_init().
 *
 * @param oled_handle A handle to the initialized OLED driver instance.
 * @note The specific command values used here are for a common 128x64 display
 * and may need to be adjusted for different hardware.
 */
void send_config_commands(oled_handle_t oled_handle) {
    
    unsigned char double_cmd[3] = {0x00, 0x00, 0x00};
    unsigned char single_cmd[2] = {0x00, 0x00};
    if (xSemaphoreTake(oled_handle->i2c_mutex, portMAX_DELAY) == pdTRUE) {
        
        // Turn Display OFF (0xAE)
        ESP_LOGI(TAG, "Turning Display OFF");
        single_cmd[0] = 0x00;
        single_cmd[1] = 0xAE;
        send_command(oled_handle, single_cmd, 2);
        

        // Set Display Clock Divide Ratio/Oscillator Frequency (0xD5)
        ESP_LOGI(TAG, "Setting Display Clock Divide Ratio/Oscillator Frequency");
        double_cmd[0] = 0x00;
        double_cmd[1] = 0xD5;
        double_cmd[2] = 0x80;
        send_command(oled_handle, double_cmd, 3);

        // Set Multiplex Ratio (0xA8) to 64MUX
        ESP_LOGI(TAG, "Setting Multiplex Ratio (0xA8) to 64MUX");
        double_cmd[0] = 0x00;
        double_cmd[1] = 0xA8;
        double_cmd[2] = 0x3F; // 63d for 64 MUX
        send_command(oled_handle, double_cmd, 3);

        // Set Display Offset (0xD3) with no offset
        ESP_LOGI(TAG, "Setting Display Offset (0xD3) with no offset");
        double_cmd[0] = 0x00;
        double_cmd[1] = 0xD3;
        double_cmd[2] = 0x00;
        send_command(oled_handle, double_cmd, 3);

        // Set Display Start Line (0x40) to line 0
        ESP_LOGI(TAG, "Setting Display Start Line (0x40) to line 0");
        single_cmd[0] = 0x00;
        single_cmd[1] = 0x40;
        send_command(oled_handle, single_cmd, 2);

        // Enable Charge Pump Regulator (0x8D)
        ESP_LOGI(TAG, "Enabling Charge Pump Regulator (0x8D)");
        double_cmd[0] = 0x00;
        double_cmd[1] = 0x8D;
        double_cmd[2] = 0x14;
        send_command(oled_handle, double_cmd, 3);
        
        // Set Memory Addressing Mode (0x20) to Horizontal Mode
        ESP_LOGI(TAG, "Setting Memory Addressing Mode (0x20) to Horizontal Mode");
        double_cmd[0] = 0x00;
        double_cmd[1] = 0x20;
        double_cmd[2] = 0x00;
        send_command(oled_handle, double_cmd, 3);

        // Set Segment Re-map (0xA1), mapping column 127 to SEG0
        ESP_LOGI(TAG, "Setting Segment Re-map (0xA1), mapping column 127 to SEG0");
        single_cmd[0] = 0x00;
        single_cmd[1] = 0xA1;
        send_command(oled_handle, single_cmd, 2);

        // Set COM Output Scan Direction (0xC8) in remapped mode
        ESP_LOGI(TAG, "Setting COM Output Scan Direction (0xC8) in remapped mode");
        single_cmd[0] = 0x00;
        single_cmd[1] = 0xC8;
        send_command(oled_handle, single_cmd, 2);

        // Set COM Pins Hardware Configuration (0xDA)
        ESP_LOGI(TAG, "Setting COM Pins Hardware Configuration (0xDA)");
        double_cmd[0] = 0x00;
        double_cmd[1] = 0xDA;
        double_cmd[2] = 0x12;
        send_command(oled_handle, double_cmd, 3);

        // Set Contrast Control (0x81)
        ESP_LOGI(TAG, "Setting Contrast Control (0x81)");
        double_cmd[0] = 0x00;
        double_cmd[1] = 0x81;
        double_cmd[2] = 0xCF;
        send_command(oled_handle, double_cmd, 3);

        // Set Pre-charge Period (0xD9)
        ESP_LOGI(TAG, "Setting Pre-charge Period (0xD9)");
        double_cmd[0] = 0x00;
        double_cmd[1] = 0xD9;
        double_cmd[2] = 0xF1;
        send_command(oled_handle, double_cmd, 3);
        
        // Set VCOMH Deselect Level (0xDB)
        ESP_LOGI(TAG, "Setting VCOMH Deselect Level (0xDB)");
        double_cmd[0] = 0x00;
        double_cmd[1] = 0xDB;
        double_cmd[2] = 0x40;
        send_command(oled_handle, double_cmd, 3);
        
        // Resume display from GDDRAM content (0xA4)
        ESP_LOGI(TAG, "Resuming display from GDDRAM content (0xA4)");
        single_cmd[0] = 0x00;
        single_cmd[1] = 0xA4;
        send_command(oled_handle, single_cmd, 2);

        // Set Normal Display mode (0xA6)
        ESP_LOGI(TAG, "Setting Normal Display mode (0xA6)");
        single_cmd[0] = 0x00;
        single_cmd[1] = 0xA6;
        send_command(oled_handle, single_cmd, 2);
        
        // Turn Display ON (0xAF)
        ESP_LOGI(TAG, "Turning Display ON (0xAF)");
        single_cmd[0] = 0x00;
        single_cmd[1] = 0xAF;
        send_command(oled_handle, single_cmd, 2);
        
        // Clear the screen memory after configuration
        ESP_LOGI(TAG, "Clearing screen memory after configuration");
        xSemaphoreGive(oled_handle->i2c_mutex);
        oled_flush_gddram(oled_handle);
        
    }
}
    
    /**
     * @brief Performs a simple visual test of the OLED screen by flashing it.
     * @details This function is typically called once after initialization to confirm
     * that the display is connected and responding to commands. It cycles the
     * display between "Entire Display ON" mode and normal GDDRAM mode multiple
     * times, creating a flashing effect.
     *
     * @param oled_handle A handle to the initialized OLED driver instance.
 */
void test_oled(oled_handle_t oled_handle) {
    unsigned char command[2] = {0, 0};
    int delay = 100;
    ESP_LOGI("OLED", "Testing OLED. Screen should be flashing.");
    if(xSemaphoreTake(oled_handle->i2c_mutex, portMAX_DELAY) == pdTRUE) {
        for (int i = 0; i < 10; i++) {
            // Command 0xA5: Force entire display ON, ignoring GDDRAM content.
            command[0] = 0x00;
            command[1] = 0xA5;
            ESP_ERROR_CHECK(i2c_master_transmit(oled_handle->i2c_device_oled_handle, command, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
            vTaskDelay(delay / portTICK_PERIOD_MS);
    
            // Command 0xA4: Resume display to follow GDDRAM content.
            command[0] = 0x00;
            command[1] = 0xA4;
            ESP_ERROR_CHECK(i2c_master_transmit(oled_handle->i2c_device_oled_handle, command, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
            vTaskDelay(delay / portTICK_PERIOD_MS);
        }
        xSemaphoreGive(oled_handle->i2c_mutex);
    }
    ESP_LOGI("OLED", "Finished testing OLED.");
}


/**
 * @brief Updates the physical OLED screen with the contents of the local framebuffer.
 * @details This function takes the entire local framebuffer, which is stored in the
 * oled_handle, and transmits it to the display's GDDRAM. This is the
 * final step that makes any changes made by drawing functions (like
 * oled_toggle_pixel or oled_square_filled) visible on the screen.
 *
 * @param oled_handle A handle to the initialized OLED driver instance, which
 * contains the framebuffer to be displayed.
 */
void oled_update_frame_buffer(oled_handle_t oled_handle) {
    // TODO Make this a task and only allocate memory once

    int bitmap_size = (oled_handle->width * oled_handle->height) / 8;
    int write_buf_size = bitmap_size + 1;

    unsigned char* write_buf = (unsigned char*)malloc(write_buf_size);

    // size_t free_heap = esp_get_free_heap_size();
    // ESP_LOGI(TAG, "Free heap before buffer update: %d bytes", free_heap);

    if (write_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for frame buffer update!");
        return; // Exit the function immediately
    }
    // The first byte of the transmission must be the data command byte (0x40)
    write_buf[0] = 0x40;
    
    // Copy the local framebuffer data into the transmission buffer
    memcpy(&write_buf[1], oled_handle->frame_buffer, bitmap_size);
    
    // Ensure the display's memory pointer is at the start (0,0) before writing
    reset_cursor(oled_handle);
    
    if (xSemaphoreTake(oled_handle->i2c_mutex, portMAX_DELAY) == pdTRUE) {
        // Transmit the complete buffer to the OLED
        ESP_ERROR_CHECK(i2c_master_transmit(oled_handle->i2c_device_oled_handle, write_buf, write_buf_size, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
        xSemaphoreGive(oled_handle->i2c_mutex);
    }
    
    
    free(write_buf);
}

/**
 * @brief Initializes the OLED driver and hardware.
 * @details This function allocates memory for the driver's internal state, including
 * the framebuffer, configures the I2C communication for the display, sends the
 * necessary initialization commands to the SSD1306 controller, and performs a
 * brief startup test.
 *
 * @param pBus_handle Pointer to the I2C master bus handle, which must be initialized
 * before calling this function.
 * @param oled_cfg    Pointer to a configuration struct containing all necessary
 * settings for the OLED display.
 * @return            A handle to the OLED driver instance on success, or NULL if
 * memory allocation fails.
 */
oled_handle_t oled_init(i2c_master_bus_handle_t bus_handle, oled_config_t* oled_cfg) {
    // Allocate memory for the main driver structure
    oled_handle_t handle = (oled_handle_t)malloc(sizeof(struct oled));
    if (handle == NULL) {
        ESP_LOGI("OLED", "oled handle malloc failed");
        return NULL;
    }
    
    // Copy configuration parameters into the handle
    handle->bus_handle = bus_handle;
    handle->device_address = oled_cfg->device_address;
    handle->address_bit_length = oled_cfg->address_bit_length;
    handle->width = oled_cfg->width;
    handle->height = oled_cfg->height;
    handle->refresh_rate_ms = oled_cfg->refresh_rate_ms;
    handle->i2c_device_oled_handle = oled_cfg->i2c_device_oled_handle;
    handle->scl_speed_hz = oled_cfg->scl_speed_hz;
    handle->i2c_mutex = oled_cfg->i2c_mutex;
    
    // Allocate and clear the local framebuffer
    int buffer_size = (handle->width * handle->height) / 8;
    handle->frame_buffer = (unsigned char*)malloc(buffer_size);
    if (handle->frame_buffer == NULL) {
        ESP_LOGI("OLED", "oled framebuffer malloc failed");
        free(handle); // Clean up the handle if framebuffer allocation fails
        return NULL;
    }
    memset(handle->frame_buffer, 0x00, buffer_size);
    
    // Configure the I2C device settings
    i2c_device_config_t i2c_device_oled_cfg = {
        .dev_addr_length = oled_cfg->address_bit_length,
        .device_address = oled_cfg->device_address,
        .scl_speed_hz = oled_cfg->scl_speed_hz,
    };
    
    // Add the OLED as a new device on the I2C bus
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &i2c_device_oled_cfg, &handle->i2c_device_oled_handle));
    
    // Send initialization commands to the OLED controller
    send_config_commands(handle);
    
    // Perform a visual test to confirm initialization
    test_oled(handle);

    return handle;
}

/**
 * UNDER CONSTRUCTION
 */
void oled_print_bitmap(oled_handle_t oled_handle, const unsigned char* pBitmap, int bitmap_size){
    
    
    int write_buf_size = (oled_handle->width * oled_handle->height)/8 +1;
    unsigned char* write_buf = (unsigned char* )malloc(write_buf_size);

    write_buf[0] = 0x40;
    // This operation prepares the bitmap format for display
    // TODO Need to see about bitmap size and check if fits the oled
    //bitmap_to_page_buffer(oled_handle, pBitmap, write_buf, write_buf_size);
    memcpy(&write_buf[1], pBitmap, bitmap_size);
    // Set the Column and Page addresses
    reset_cursor(oled_handle);

    if (xSemaphoreTake(oled_handle->i2c_mutex, portMAX_DELAY) == pdTRUE) {
        // Send the data to the OLED display
        ESP_ERROR_CHECK(i2c_master_transmit(oled_handle->i2c_device_oled_handle, write_buf, write_buf_size, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
        xSemaphoreGive(oled_handle->i2c_mutex);
    }

    free(write_buf);
};

/**
 * @brief Draws a filled rectangle on the OLED display.
 * @details This function modifies the local framebuffer to draw a solid rectangle
 * defined by two opposing corner coordinates. It iterates through every pixel
 * within these bounds and sets the corresponding bit in the framebuffer to 'on'.
 * The function handles any corner combination (e.g., top-left to bottom-right,
 * or bottom-left to top-right).
 *
 * @note This function draws to a local framebuffer and then calls 
 * oled_update_frame_buffer() to push the changes to the physical screen. If
 * drawing multiple shapes, it can be more efficient to create a separate
 * function that doesn't call the update automatically.
 *
 * @param oled_handle A handle to the initialized OLED driver instance.
 * @param x1 The x-coordinate of the first corner of the rectangle (0-127).
 * @param y1 The y-coordinate of the first corner of the rectangle (0-63).
 * @param x2 The x-coordinate of the second corner of the rectangle (0-127).
 * @param y2 The y-coordinate of the second corner of the rectangle (0-63).
 */
void oled_square_filled(oled_handle_t oled_handle, int x1, int y1, int x2, int y2) {
    // Determine the rectangle's boundaries
    int lower_x = x1 > x2 ? x2 : x1;
    int upper_x = x1 > x2 ? x1 : x2;
    int lower_y = (y1 > y2 ? y2 : y1);
    int upper_y = (y1 > y2 ? y1 : y2);

    // This function modifies the local framebuffer, not the OLED directly
    for (int x = lower_x; x <= upper_x; x++) {
        for (int y = lower_y; y <= upper_y; y++) {
            // Calculate the position in the framebuffer
            int page = y / 8;
            int bit = y % 8;
            int index = x + (page * oled_handle->width);
            
            // Set the bit in the framebuffer using a bitwise OR operation.
            // This ensures other pixels in the same byte are not affected.
            // Change to XOR(^) operation to toggle pixel on passing
            oled_handle->frame_buffer[index] |= (1 << bit);
        }
    }
}

void oled_circle_filled(oled_handle_t oled_handle, int x_center, int y_center, int radius) {
    // Determine the rectangle's boundaries
    int lower_x = x_center - radius;
    int upper_x = x_center + radius;
    int lower_y = y_center - radius;
    int upper_y = y_center + radius;
    float r_squared = (float)(radius * radius);

    // This function modifies the local framebuffer, not the OLED directly
    for (int i = lower_x; i <= upper_x; i++) {
        for (int j = lower_y; j <= upper_y; j++) {

            if ((float)((j-y_center) * (j-y_center)) + (float)((i-x_center)*(i-x_center)) <= r_squared) {
                int page = j / 8;
                int bit = j % 8;
                int index = i + (page * oled_handle->width);
    
                // Set the bit in the framebuffer using a bitwise OR operation.
                // This ensures other pixels in the same byte are not affected.
                // Change to XOR(^) operation to toggle pixel on passing
                oled_handle->frame_buffer[index] |= (1 << bit);
                // Calculate the position in the framebuffer
            }
        }
    }
}

void oled_clean_screen(oled_handle_t oled_handle) {
    // Clear the framebuffer
    memset(oled_handle->frame_buffer, 0, oled_handle->width * oled_handle->height / 8);
}