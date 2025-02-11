#ifndef MPU9250_H
#define MPU9250_H
#include "mpu6050.h"


#include <esp_err.h>
#include <i2cdev.h>

#define AK8963_I2C_ADDRESS  0x0C
#define TWO_COMPLEMENT_SIGN_MASK ((uint16_t)1<<15)
#ifdef __cplusplus
extern "C" {
#endif

typedef struct __attribute__((packed))
{
  int16_t x;
  int16_t y;
  int16_t z;
} ak8963_raw_magnetometer_t;

/**
 * AK8963 magnetic data, uT
 */
typedef struct __attribute__((packed))
{
  float x;
  float y;
  float z;
} ak8963_magnetometer_t;

typedef struct __attribute__((packed))
{
  uint8_t data_ready:1;
  uint8_t data_miss:1;
  uint8_t null0:6;
  ak8963_raw_magnetometer_t raw_mag;
  uint8_t null1:3;
  uint8_t sensor_overflow:1;
  uint8_t output_mode:1;
  uint8_t null2:3;
  uint8_t control1;
} ak8963_read_chunk_t;


typedef enum {
  AK8963_POWER_DOWN = 0,
  AK8963_SINGLE_MEASUREMENT,
  AK8963_CONTINUOUS_MEASUREMENT_1,
  AK8963_CONTINUOUS_MEASUREMENT_2,
  AK8963_EXTERNAL_TRIGGER,
  AK8963_SELF_TEST,
  AK8963_FUSE_ACCESS,
  AK8963_MAX_MODES
} ak8963_mode_t;

/**
 * Device descriptor
 */
typedef struct{
  mpu6050_dev_t mpu6050_dev;
  i2c_dev_t i2c_dev; //ak8963 i2c
} mpu9250_dev_t;

/**
 * @brief Initialize device descriptor.
 *
 * @param dev Device descriptor
 * @param addr Device I2C address
 * @param port I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 *
 * @return `ESP_OK` on success
 */
esp_err_t mpu9250_init_desc(mpu9250_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor.
 *
 * @param dev Device descriptor
 *
 * @return `ESP_OK` on success
 */
esp_err_t mpu9250_free_desc(mpu9250_dev_t *dev);

/**
 * @brief Initialize device.
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t mpu9250_init(mpu9250_dev_t *dev);


/**
 * @brief Get 3-axis magnetometer readings.
 *
 *
 * @param dev Device descriptor
 * @param[out] mag Three-axis mag data, uT.
 *
 * @return `ESP_OK` on success
 */
esp_err_t mpu9250_get_mag(mpu9250_dev_t *dev, ak8963_magnetometer_t*mag);

/**
 * @brief Get raw 3-axis magnetometer readings.
 *
 * @param dev Device descriptor
 * @param[out] raw_mag Raw magnetometer data.
 *
 * @return `ESP_OK` on success
 */
esp_err_t mpu9250_get_raw_mag(mpu9250_dev_t *dev, ak8963_raw_magnetometer_t *raw_mag);

/**
 * @brief Get raw 9-axis motion sensor readings (accel/gyro/mag).
 *
 * Retrieves all currently available motion sensor values.
 *
 * @param dev Device descriptor
 * @param[out] data_accel acceleration struct.
 * @param[out] data_gyro rotation struct.
 * @param[out] data_mag magnetometer struct.
 *
 * @return `ESP_OK` on success
 */
esp_err_t mpu9250_get_motion(mpu9250_dev_t *dev, mpu6050_acceleration_t *data_accel, mpu6050_rotation_t *data_gyro, ak8963_magnetometer_t *data_mag);



/**
 * @brief Performs basic tests on chip
 *
 * @param addr Device address
 * @param sda SDA I2C Pin
 * @param scl SCL I2C Pin
 * 
 * @return `ESP_OK` on success
 */
esp_err_t mpu9250_tests(mpu9250_dev_t *dev, uint8_t addr, gpio_num_t sda, gpio_num_t scl);


#ifdef __cplusplus
}
#endif


#endif