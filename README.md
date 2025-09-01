# ESP32 Digital Level

A high-precision digital level built using an ESP32, an MPU9250 9-axis IMU, and an I2C OLED display. This project leverages a sophisticated sensor fusion algorithm to provide a smooth and accurate visualization of orientation in real-time.

The application reads data from the accelerometer, gyroscope, and magnetometer, fuses it using a Mahony filter to compute a stable orientation quaternion, and then translates this data into a visual "bubble" on an OLED screen.

![gif](https://github.com/user-attachments/assets/118fc69b-7c7a-46c3-be60-b85578c76364)

## ✨ Features

  * **Real-time Orientation Sensing**: Utilizes an MPU9250 Inertial Measurement Unit to capture 9-axis motion data.
  * **Advanced Sensor Fusion**: Implements a **Mahony filter** to combine sensor data, correcting for drift and noise to produce a stable orientation quaternion ($q = w + xi + yj + zk$).
  * **Clear Visual Feedback**: Displays the level on a 128x64 OLED screen, representing the tilt with a circle on a central crosshair.
  * **Robust Drivers**: Includes custom, well-documented drivers for both the MPU9250 sensor and the SSD1306 OLED display.
  * **RTOS-Powered**: Built on FreeRTOS for efficient task management, separating sensor data processing from display rendering.
  * **Highly Configurable**: Key parameters like I2C pins and device addresses are easily configurable through macros.

## 🛠️ Hardware Requirements

| Component              | Description                                      |
| ---------------------- | ------------------------------------------------ |
| **Microcontroller** | ESP32 Development Board                          |
| **Inertial Sensor** | MPU9250 9-Axis (Gyro + Accel + Mag) Module       |
| **Display** | 128x64 I2C OLED Display (SSD1306 controller)     |
| **Wiring** | Breadboard and jumper wires                      |
| **Power** | 3.3V power source (usually from the ESP32 board) |

## 🔌 Wiring Diagram

Connect the components to the ESP32 using the I2C bus. The default pins are configured in `main.c`.

| ESP32 Pin   | MPU9250 Pin | OLED Display Pin |
| ----------- | ----------- | ---------------- |
| `GPIO 22`   | `SCL`       | `SCL`            |
| `GPIO 21`   | `SDA`       | `SDA`            |
| `3V3`       | `VCC`       | `VCC`            |
| `GND`       | `GND`       | `GND`            |

## ⚙️ Configuration

You can easily modify the core hardware settings by changing the `#define` macros at the top of the `main.c` file.

```c
// main.c

// I2C CONFIG
#define SCL_SPEED_HZ 400000
#define SCL_PIN GPIO_NUM_22
#define SDA_PIN GPIO_NUM_21

// OLED CONFIG
#define OLED_I2C_ADDRESS 0x3c
#define OLED_SCREEN_WIDTH 128
#define OLED_SCREEN_HEIGHT 64
#define OLED_REFRESH_RATE_MS 250

// MPU9250 CONFIG
#define MPU9250_SENSOR_ADDR 0x68
```

## 🚀 Installation & Usage

This project is built using the Espressif IoT Development Framework (ESP-IDF).

1.  **Clone the Repository**:

    ```bash
    git clone <repository-url>
    cd esp32-digital-level
    ```

2.  **Configure the Project**:
    Set your target ESP32 chip and serial port.

    ```bash
    idf.py set-target esp32
    idf.py menuconfig
    ```

    (No specific project configuration is required in `menuconfig`, but you can use it to set up your serial flasher options).

3.  **Build the Project**:

    ```bash
    idf.py build
    ```

4.  **Flash and Monitor**:
    Connect your ESP32 board, then flash the firmware and start the serial monitor.

    ```bash
    idf.py flash monitor
    ```

Upon startup, the OLED display will flash briefly to confirm it's working, and then the digital level interface will appear.

## 🔬 How It Works

The system operates through two primary FreeRTOS tasks that run concurrently.

1.  **MPU9250 Sensor Task (`mpu9250_task`)**:

      * This task continuously polls the MPU9250 sensor for raw accelerometer, gyroscope, and magnetometer data.
      * It applies a **Mahony fusion filter** to this raw data. The filter uses the gyroscope data as its primary source for orientation changes and corrects for its inherent drift using the accelerometer (for gravity vector) and magnetometer (for Earth's magnetic field) data.
      * The output of the filter is a normalized **quaternion**, which represents the sensor's orientation in 3D space without the risk of gimbal lock.
      * This quaternion is stored in the MPU9250 driver handle, ready to be requested by other tasks.

2.  **Display Task (`level_task`)**:

      * This is the main application logic loop.
      * It requests the latest orientation quaternion from the MPU9250 driver.
      * The quaternion ($q$) is converted into more intuitive **Euler angles** (pitch, roll, yaw) using the following formulas:
        $$Roll = \arctan(2(q_w q_x + q_y q_z) / 1 - 2(q_x^2 + q_y^2))$$ </br>
        $$Pitch = \arcsin(2(q_w q_y - q_z q_x))$$
      * The `pitch` and `roll` values are then mapped to the `x` and `y` coordinates of the OLED display.
      * Finally, the task clears the screen, draws a static crosshair, and draws a circle at the calculated `(x, y)` position to represent the level's "bubble". This entire framebuffer is then pushed to the OLED display.

## 📂 Code Overview

  * **`main.c`**: The main application entry point. It handles the initialization of the I2C bus, the MPU9250 driver, and the OLED driver. It also creates and starts the `level_task`.
  * **`oled_driver.c/.h`**: A modular driver for the SSD1306 OLED display. It provides an API for initialization, drawing shapes (filled squares, filled circles), clearing the screen, and updating the display from a local framebuffer.
  * **`mpu9250_driver.c/.h`**: A comprehensive driver for the MPU9250 IMU. It manages I2C communication, sensor configuration, and runs a background task to perform sensor fusion and calculate orientation.
  * **`mpu9240_register_map.h`**: A header file containing all the I2C register addresses for the MPU9250 and its internal AK8963 magnetometer, making the driver code clean and readable.

## 🚧 Future Improvements

  * [ ] **Runtime Calibration**: Implement a calibration routine at startup to calculate and correct for gyroscope and magnetometer biases.
  * [ ] **Enhanced Visualization**: Create a more graphical "bubble level" interface instead of a simple circle and crosshair.
  * [ ] **Power Optimization**: Use the MPU9250's low-power modes and FreeRTOS tickless idle for battery-powered applications.
  * [ ] **Refactor Display Update**: As noted in the `oled_update_frame_buffer` `TODO`, move the memory allocation to a one-time operation to improve performance and reduce heap fragmentation.
