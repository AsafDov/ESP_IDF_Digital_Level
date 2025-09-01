#pragma once
/**
 * @file mpu9250_register_map.h
 * @brief MPU-9250 register map definitions
 */

/* Gyroscope and Accelerometer Self-Test Registers */
#define MPU9250_SELF_TEST_X_GYRO_REG_ADDR     0x00  /*!< X-axis gyroscope self-test register */
#define MPU9250_SELF_TEST_Y_GYRO_REG_ADDR     0x01  /*!< Y-axis gyroscope self-test register */
#define MPU9250_SELF_TEST_Z_GYRO_REG_ADDR     0x02  /*!< Z-axis gyroscope self-test register */
#define MPU9250_SELF_TEST_X_ACCEL_REG_ADDR    0x0D  /*!< X-axis accelerometer self-test register */
#define MPU9250_SELF_TEST_Y_ACCEL_REG_ADDR    0x0E  /*!< Y-axis accelerometer self-test register */
#define MPU9250_SELF_TEST_Z_ACCEL_REG_ADDR    0x0F  /*!< Z-axis accelerometer self-test register */

/* Gyroscope Offset Registers */
#define MPU9250_XG_OFFSET_H_REG_ADDR          0x13  /*!< Gyroscope X-axis offset, high byte */
#define MPU9250_XG_OFFSET_L_REG_ADDR          0x14  /*!< Gyroscope X-axis offset, low byte */
#define MPU9250_YG_OFFSET_H_REG_ADDR          0x15  /*!< Gyroscope Y-axis offset, high byte */
#define MPU9250_YG_OFFSET_L_REG_ADDR          0x16  /*!< Gyroscope Y-axis offset, low byte */
#define MPU9250_ZG_OFFSET_H_REG_ADDR          0x17  /*!< Gyroscope Z-axis offset, high byte */
#define MPU9250_ZG_OFFSET_L_REG_ADDR          0x18  /*!< Gyroscope Z-axis offset, low byte */

/* Configuration Registers */
#define MPU9250_SMPLRT_DIV_REG_ADDR           0x19  /*!< Sample Rate Divider register */
#define MPU9250_CONFIG_REG_ADDR               0x1A  /*!< Configuration register */
#define MPU9250_GYRO_CONFIG_REG_ADDR          0x1B  /*!< Gyroscope Configuration register */
#define MPU9250_ACCEL_CONFIG_REG_ADDR         0x1C  /*!< Accelerometer Configuration register */
#define MPU9250_ACCEL_CONFIG_2_REG_ADDR       0x1D  /*!< Accelerometer Configuration 2 register */
#define MPU9250_LP_ACCEL_ODR_REG_ADDR         0x1E  /*!< Low Power Accelerometer ODR Control register */
#define MPU9250_WOM_THR_REG_ADDR              0x1F  /*!< Wake-on Motion Threshold register */

/* FIFO Registers */
#define MPU9250_FIFO_EN_REG_ADDR              0x23  /*!< FIFO Enable register */
#define MPU9250_FIFO_COUNTH_REG_ADDR          0x72  /*!< FIFO Count, high byte register */
#define MPU9250_FIFO_COUNTL_REG_ADDR          0x73  /*!< FIFO Count, low byte register */
#define MPU9250_FIFO_R_W_REG_ADDR             0x74  /*!< FIFO Read Write register */

/* I2C Slave 0 Control Registers */
#define MPU9250_I2C_SLV0_ADDR_REG_ADDR       0x25  /*!< I2C Slave 0 Address register */
#define MPU9250_I2C_SLV0_REG_ADDR            0x26  /*!< I2C Slave 0 Register register */
#define MPU9250_I2C_SLV0_CTRL_REG_ADDR       0x27  /*!< I2C Slave 0 Control register */

/* Interrupt Registers */
#define MPU9250_INT_PIN_CFG_REG_ADDR          0x37  /*!< INT Pin / Bypass Enable Configuration register */
#define MPU9250_INT_ENABLE_REG_ADDR           0x38  /*!< Interrupt Enable register */
#define MPU9250_INT_STATUS_REG_ADDR           0x3A  /*!< Interrupt Status register */

/* Accelerometer Data Registers */
#define MPU9250_ACCEL_XOUT_H_REG_ADDR         0x3B  /*!< Accelerometer X-axis measurement, high byte */
#define MPU9250_ACCEL_XOUT_L_REG_ADDR         0x3C  /*!< Accelerometer X-axis measurement, low byte */
#define MPU9250_ACCEL_YOUT_H_REG_ADDR         0x3D  /*!< Accelerometer Y-axis measurement, high byte */
#define MPU9250_ACCEL_YOUT_L_REG_ADDR         0x3E  /*!< Accelerometer Y-axis measurement, low byte */
#define MPU9250_ACCEL_ZOUT_H_REG_ADDR         0x3F  /*!< Accelerometer Z-axis measurement, high byte */
#define MPU9250_ACCEL_ZOUT_L_REG_ADDR         0x40  /*!< Accelerometer Z-axis measurement, low byte */

/* Temperature Data Registers */
#define MPU9250_TEMP_OUT_H_REG_ADDR           0x41  /*!< Temperature measurement, high byte */
#define MPU9250_TEMP_OUT_L_REG_ADDR           0x42  /*!< Temperature measurement, low byte */

/* Gyroscope Data Registers */
#define MPU9250_GYRO_XOUT_H_REG_ADDR          0x43  /*!< Gyroscope X-axis measurement, high byte */
#define MPU9250_GYRO_XOUT_L_REG_ADDR          0x44  /*!< Gyroscope X-axis measurement, low byte */
#define MPU9250_GYRO_YOUT_H_REG_ADDR          0x45  /*!< Gyroscope Y-axis measurement, high byte */
#define MPU9250_GYRO_YOUT_L_REG_ADDR          0x46  /*!< Gyroscope Y-axis measurement, low byte */
#define MPU9250_GYRO_ZOUT_H_REG_ADDR          0x47  /*!< Gyroscope Z-axis measurement, high byte */
#define MPU9250_GYRO_ZOUT_L_REG_ADDR          0x48  /*!< Gyroscope Z-axis measurement, low byte */

/* External Sensor Registers (Magnetometer) */
#define MPU9250_EXT_SENS_DATA_00_REG_ADDR     0x49  /*!< External Sensor Data Register 0 */
#define MPU9250_EXT_SENS_DATA_01_REG_ADDR     0x4A  /*!< External Sensor Data Register 1 */
#define MPU9250_EXT_SENS_DATA_02_REG_ADDR     0x4B  /*!< External Sensor Data Register 2 */
#define MPU9250_EXT_SENS_DATA_03_REG_ADDR     0x4C  /*!< External Sensor Data Register 3 */
#define MPU9250_EXT_SENS_DATA_04_REG_ADDR     0x4D  /*!< External Sensor Data Register 4 */
#define MPU9250_EXT_SENS_DATA_05_REG_ADDR     0x4E  /*!< External Sensor Data Register 5 */
#define MPU9250_EXT_SENS_DATA_06_REG_ADDR     0x4F  /*!< External Sensor Data Register 6 */

/* Control and Management Registers */
#define MPU9250_USER_CTRL_REG_ADDR            0x6A  /*!< User Control register */
#define MPU9250_PWR_MGMT_1_REG_ADDR           0x6B  /*!< Power Management 1 register */
#define MPU9250_PWR_MGMT_2_REG_ADDR           0x6C  /*!< Power Management 2 register */
#define MPU9250_SIGNAL_PATH_RESET_REG_ADDR    0x68  /*!< Signal Path Reset register */
#define MPU9250_WHO_AM_I_REG_ADDR             0x75  /*!< Who Am I register, returns 0x71 */

/* Magnetometer (AK8963) Registers */
#define MPU9250_MAG_DEVICE_ADDR               0x0C  /*!< Magnetometer I2C address */
#define MPU9250_MAG_WIA_REG_ADDR              0x00  /*!< Magnetometer Device ID register */
#define MPU9250_MAG_ST1_REG_ADDR              0x02  /*!< Magnetometer Status 1 register */
#define MPU9250_MAG_HXL_REG_ADDR              0x03  /*!< Magnetometer X-axis data, low byte */
#define MPU9250_MAG_HXH_REG_ADDR              0x04  /*!< Magnetometer X-axis data, high byte */
#define MPU9250_MAG_HYL_REG_ADDR              0x05  /*!< Magnetometer Y-axis data, low byte */
#define MPU9250_MAG_HYH_REG_ADDR              0x06  /*!< Magnetometer Y-axis data, high byte */
#define MPU9250_MAG_HZL_REG_ADDR              0x07  /*!< Magnetometer Z-axis data, low byte */
#define MPU9250_MAG_HZH_REG_ADDR              0x08  /*!< Magnetometer Z-axis data, high byte */
#define MPU9250_MAG_ST2_REG_ADDR              0x09  /*!< Magnetometer Status 2 register */
#define MPU9250_MAG_CNTL1_REG_ADDR            0x0A  /*!< Magnetometer Control 1 register */

/* Bit Definitions */
#define MPU9250_RESET_BIT                     7     /*!< Bit 7 in PWR_MGMT_1 register to reset device */

/* MPU Accelerometer Sensitivity */
#define MPU9250_ACCEL_SENSITIVITY_2G          16384  /*!< Sensitivity for ±2g range */
#define MPU9250_ACCEL_SENSITIVITY_4G          8192   /*!< Sensitivity for ±4g range */
#define MPU9250_ACCEL_SENSITIVITY_8G          4096   /*!< Sensitivity for ±8g range */
#define MPU9250_ACCEL_SENSITIVITY_16G         2048   /*!< Sensitivity for ±16g range */

/* MPU Gyro Sensitivity */
#define MPU9250_GYRO_SENSITIVITY_250dps      131.0f /*!< Sensitivity for ±250 degrees/sec */
#define MPU9250_GYRO_SENSITIVITY_500dps      65.5f  /*!< Sensitivity for ±500 degrees/sec */
#define MPU9250_GYRO_SENSITIVITY_1000dps     32.8f  /*!< Sensitivity for ±1000 degrees/sec */
#define MPU9250_GYRO_SENSITIVITY_2000dps     16.4f  /*!< Sensitivity for ±2000 degrees/sec */

/* Magnetometer Modes */
#define MPU9250_MAG_MODE_POWER_DOWN           0x00  /*!< Power down mode */
#define MPU9250_MAG_MODE_SINGLE_MEASUREMENT   0x11  /*!< Single measurement mode */
#define MPU9250_MAG_MODE_CONTINUOUS_8HZ       0x12  /*!< Continuous measurement mode 8Hz */
#define MPU9250_MAG_MODE_CONTINUOUS_100HZ     0x16  /*!< Continuous measurement mode 100Hz */
#define MPU9250_MAG_MODE_EXTERNAL_TRIGGER     0x14  /*!< Continuous measurement mode 50Hz */