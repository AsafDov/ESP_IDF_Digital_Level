# ESP-IDF SSD1306 OLED Driver

## 📖 About

This project is a lightweight, I2C-based driver for SSD1306 OLED displays, specifically designed for the Espressif IoT Development Framework (ESP-IDF). The goal is to provide a straightforward and efficient way to interface with these popular displays in your ESP32 projects.

Currently, the driver is in the early stages of development. The foundational functionalities, such as initialization, configuration, and basic drawing operations, are implemented. The included `main.c` provides a simple random walk example to demonstrate the driver's current capabilities.

-----

## ✨ Features

  * **I2C Communication:** Utilizes the ESP-IDF's I2C master driver for communication with the OLED display.
  * **Display Initialization:** A comprehensive initialization sequence to configure the SSD1306 controller for standard 128x64 displays.
  * **Local Framebuffer:** Implements a local framebuffer for efficient and flicker-free rendering.
  * **Basic Drawing Primitives:** Includes a function to draw filled rectangles (`oled_square_filled`), which can be used for drawing individual pixels or lines.
  * **Bitmap Support:** A function to display monochrome bitmaps is in development.

-----

## 🛠️ Hardware Requirements

  * An ESP32 development board
  * An SSD1306-based OLED display (128x64 resolution)
  * Jumper wires for connecting the display to the ESP32

### Pin Configuration

The driver is configured to use the following default pins, which can be easily changed in `main.c`:

  * **SCL:** GPIO 22
  * **SDA:** GPIO 21

-----

## 🚀 Getting Started

### Prerequisites

  * ESP-IDF v5.5 or later installed and configured.
  * A working toolchain for building and flashing ESP32 projects.

### Configuration

1.  **Clone the Repository:**
    ```bash
    git clone https://github.com/AsafDov/ESP_IDF_SSD1306_OLED_Driver/
    ```
2.  **Navigate to the Project Directory:**
    ```bash
    cd ESP_IDF_SSD1306_OLED_Driver
    ```
3.  **Configure the Project:**
    Open `main.c` and adjust the I2C pins and OLED configuration macros as needed for your hardware setup.
    ```c
    #define SCL_PIN GPIO_NUM_22
    #define SDA_PIN GPIO_NUM_21

    #define OLED_I2C_ADDRESS 0x3c
    #define OLED_SCREEN_WIDTH 128
    #define OLED_SCREEN_HEIGHT 64
    ```
4.  **Build and Flash:**
    ```bash
    idf.py build
    idf.py -p (YOUR_PORT) flash
    ```
5.  **Monitor the Output:**
    ```bash
    idf.py -p (YOUR_PORT) monitor
    ```

### Basic Usage Example
Random Walk Simulation
![ezgif com-optimize (1)](https://github.com/user-attachments/assets/4bd85a07-feb8-43f4-b65d-c78f8a4fa3af)

Here's a simplified example of how to use the driver:

```c
#include "oled_driver.h"

void app_main(void) {
    // 1. Initialize the I2C master bus
    i2c_master_bus_handle_t bus_handle;
    // ... (I2C initialization code) ...

    // 2. Configure the OLED display
    oled_config_t oled_cfg = {
        .device_address = OLED_I2C_ADDRESS,
        .width = OLED_SCREEN_WIDTH,
        .height = OLED_SCREEN_HEIGHT,
        // ... (other configuration) ...
    };

    // 3. Initialize the OLED driver
    oled_handle_t oled_handle = oled_init(&bus_handle, &oled_cfg);

    // 4. Draw to the framebuffer
    oled_square_filled(oled_handle, 10, 10, 20, 20);

    // 5. The oled_update_frame_buffer() is called within oled_square_filled()
    //    to push the changes to the display.
}
```

-----

## 📋 Driver API

The public functions for interacting with the OLED driver are defined in `oled_driver.h`.

| Function                       | Description                                                                                                                              |
| ------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------- |
| `oled_init()`                  | Initializes the OLED driver and the display hardware.                                                                                    |
| `oled_flush_gddram()`          | Clears the entire display by writing zeros to the GDDRAM.                                                                                |
| `oled_print_bitmap()`          | (Under Development) Renders a monochrome bitmap on the screen.                                                                           |
| `oled_square_filled()`         | Draws a filled rectangle on the display. Can be used to set individual pixels by providing the same start and end coordinates.             |
| `oled_get_refresh_rate()`      | Returns the configured refresh rate of the display.                                                                                      |
| `oled_update_frame_buffer()`   | Pushes the contents of the local framebuffer to the OLED display, making any changes visible.                                             |

-----

## 🚧 Development Status & Future Plans

This driver is currently a **work in progress**. While the core functionality is in place, there is still much to be done.

### Roadmap

  * [ ] **Drawing Primitives:** Implement more advanced drawing functions for lines, circles, and text.
  * [ ] **Status Bar Icons:** Implement editing of status bar with icon support
  * [ ] **Text Library:** Implement a text rendering library
  * [ ] **Font Support:** Add a font library for easy text rendering.
  * [ ] **Optimization:** Refine the I2C communication and framebuffer handling for better performance.
  * [ ] **Documentation:** Improve the in-code documentation and provide more detailed examples.

-----

## 🤝 Contributing

Contributions, issues, and feature requests are welcome\! Feel free to check the [issues page](https://www.google.com/search?q=https://github.com/your-username/your-repository/issues).
