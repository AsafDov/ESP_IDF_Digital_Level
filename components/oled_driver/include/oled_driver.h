#pragma once
/**
 * OLED driver for ESP IDF
 * Created by Asaf "Arliden The Bard"
 */

#include <stdio.h>
#include "driver/i2c_master.h"

typedef struct oled *oled_handle_t;

typedef struct oled_config{
    uint16_t device_address;
    i2c_addr_bit_len_t address_bit_length;
    int width;
    int height;
    int refresh_rate_ms;
    uint32_t scl_speed_hz;
    i2c_master_dev_handle_t i2c_device_oled_handle;
    SemaphoreHandle_t i2c_mutex;
} oled_config_t;

oled_handle_t oled_init(i2c_master_bus_handle_t bus_handle,oled_config_t* oled_cfg);
void oled_print_bitmap(oled_handle_t oled_handle, const unsigned char* bitmap, int bitmap_size );
// void oled_flush_gddram(oled_handle_t oled_handle);
void oled_square_filled(oled_handle_t oled_handle, int x1, int y1, int x2, int y2);
void oled_circle_filled(oled_handle_t oled_handle, int x, int y, int radius);
int oled_get_refresh_rate(oled_handle_t oled_handle);
void oled_update_frame_buffer(oled_handle_t oled_handle);
void oled_clean_screen(oled_handle_t oled_handle);