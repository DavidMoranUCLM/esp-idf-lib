#include "mpu6050.h"
#include "mpu6050_regs.h"
#include "mpu9250.h"
#include "ak8963_regs.h"
#include <math.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Max 1MHz for esp-idf, but device supports up to 1.7Mhz
#define I2C_FREQ_HZ (400000)

#define CHECK(x)                                                                                                                                                                                       \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        esp_err_t __;                                                                                                                                                                                  \
        if ((__ = x) != ESP_OK)                                                                                                                                                                        \
            return __;                                                                                                                                                                                 \
    }                                                                                                                                                                                                  \
    while (0)
#define CHECK_ARG(VAL)                                                                                                                                                                                 \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        if (!(VAL))                                                                                                                                                                                    \
            return ESP_ERR_INVALID_ARG;                                                                                                                                                                \
    }                                                                                                                                                                                                  \
    while (0)

static const char *TAG = "mpu9250";

static uint8_t ak8963_mode_values[] = {
    [AK8963_POWER_DOWN] = 0,
    [AK8963_SINGLE_MEASUREMENT] = 1,
    [AK8963_CONTINUOUS_MEASUREMENT_1] = 2,
    [AK8963_CONTINUOUS_MEASUREMENT_2] = 6,
    [AK8963_EXTERNAL_TRIGGER] = 4,
    [AK8963_SELF_TEST] = 8,
    [AK8963_FUSE_ACCESS] = 15,
};

static int16_t mag_adj[3];

static const float mag_res = 4912.f / 32760.f;

inline static float get_mag_value(int16_t raw);
{
    return (float)raw * mag_res;
}

inline static int16_t get_adj_raw_mag(int16_t raw, mpu6050_axis_t axis){
  return ((((mag_adj[axis]-128)*0.5)/128)+1)*raw;
}
//////////////////

inline static int8_t two_complement_is_neg(uint16_t word){
  return (~word & TWO_COMPLEMENT_SIGN_MASK) >> 15;
}

inline static int16_t two_complement_2_bin(uint16_t word){
  //uint16_t abs_word = (word << 1) >> 1 
  if (two_complement_is_neg(word)){
    return -((~word + 1) & 0xFFFF);
  }
  else{
    return word;
  }
}

inline static int16_t shuffle(uint16_t word)
{
    return (int16_t)((word >> 8) | (word << 8));
}

inline static uint16_t ushuffle(uint16_t word)
{
    return ((word >> 8) | (word << 8));
}

static esp_err_t read_reg_bits(mpu9250_dev_t *dev, uint8_t reg_addr, uint8_t offset, uint8_t mask, uint8_t *value)
{
    CHECK_ARG(dev && value && mask);

    uint8_t buf;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, reg_addr, &buf, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    *value = (buf & mask) >> offset;

    return ESP_OK;
}

static esp_err_t write_reg_bits(mpu9250_dev_t *dev, uint8_t reg_addr, uint8_t offset, uint8_t mask, uint8_t value)
{
    CHECK_ARG(dev && mask);

    uint8_t buf;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, reg_addr, &buf, 1));

    buf = (buf & ~mask) | ((value << offset) & mask);

    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, reg_addr, &buf, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static esp_err_t read_reg(mpu9250_dev_t *dev, uint8_t reg_addr, uint8_t *value)
{
    CHECK_ARG(dev && value);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, reg_addr, value, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static esp_err_t write_reg(mpu9250_dev_t *dev, uint8_t reg_addr, uint8_t data)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_write_reg(&dev->i2c_dev, reg_addr, &data, 1));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static esp_err_t read_reg_word(mpu9250_dev_t *dev, uint8_t reg_addr, int16_t *value)
{
    CHECK_ARG(dev && value);

    uint16_t buf;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, reg_addr, &buf, 2));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    *value = shuffle(buf);

    return ESP_OK;
}

static esp_err_t write_reg_word(mpu9250_dev_t *dev, uint8_t reg_addr, int16_t value)
{
    CHECK_ARG(dev);

    uint16_t buf = ushuffle(value);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, reg_addr, &buf, 2));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

static esp_err_t read_reg_bool(mpu9250_dev_t *dev, uint8_t reg_addr, uint8_t offset, bool *value)
{
    CHECK_ARG(value);

    uint8_t buf;
    CHECK(read_reg_bits(dev, reg_addr, offset, BIT(offset), &buf));
    *value = buf != 0 ? true : false;

    return ESP_OK;
}

static inline esp_err_t write_reg_bool(mpu6050_dev_t *dev, uint8_t reg_addr, uint8_t offset, bool value)
{
    return write_reg_bits(dev, reg_addr, offset, BIT(offset), value ? 1 : 0);
}

////////////////////////////////////////

static esp_err_t mpu9250_get_mag_adj_values(mpu9250_dev_t *dev)
{
    CHECK(mpu9250_set_mag_mode(dev, AK8963_POWER_DOWN));
    CHECK(mpu9250_set_mag_mode(dev, AK8963_FUSE_ACCESS));
    CHECK(read_reg_word(dev, ASAX_ADDR, &mag_adj[0]));
    CHECK(read_reg_word(dev, ASAY_ADDR, &mag_adj[1]));
    CHECK(read_reg_word(dev, ASAZ_ADDR, &mag_adj[2]));

    return ESP_OK;
}

static esp_err_t mpu9250_set_mag_mode(mpu9250_dev_t *dev, ak8963_mode_t mode)
{
    return write_reg_bits(dev, AK8963_CNTL1_ADDR, 0, AK8963_CNTL1_MODE_MASK, ak8963_mode_values[mode]);
}

static esp_err_t mpu9250_set_mag_out_bits(mpu9250_dev_t *dev, bool mode)
{
    return write_reg_bits(dev, AK8963_CNTL1_ADDR, 0, AK8963_CNTL1_BIT_MASK, mode);
}

esp_err_t mpu9250_init_desc(mpu9250_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK(mpu6050_init_desc(dev->mpu6050_dev, addr, port, sda_gpio, scl_gpio);

    CHECK_ARG(dev);

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = AK8963_I2C_ADDRESS;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t mpu9250_free_desc(mpu9250_dev_t *dev)
{
    CHECK(mpu6050_free_desc(dev->mpu6050_dev));
    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t mpu9250_init(mpu9250_dev_t *dev)
{
    CHECK(mpu6050_init(dev->mpu6050_dev));
    CHECK(mpu6050_set_i2c_master_mode_enabled(dev->mpu6050_dev, 0));
    CHECK(mpu6050_set_i2c_bypass_enabled(dev->mpu6050_dev, 1));
    CHECK(mpu9250_get_mag_adj_values(dev));
    CHECK(mpu9250_set_mag_out_bits(dev, 1));
    CHECK(mpu9250_set_mag_mode(dev, AK8963_CONTINUOUS_MEASUREMENT_2));

    return ESP_OK;
}


esp_err_t mpu9250_get_raw_mag(mpu6050_dev_t *dev, ak8963_raw_magnetometer_t *raw_mag){
  CHECK_ARG(dev && raw_mag);

  ak8963_read_chunk_t chunk;

  I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
  I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, AK8963_ST1_ADDR, chunk, sizeof(chunk)));
  I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

  raw_mag->x = two_complement_2_bin(shuffle(chunk.raw_mag.x));
  raw_mag->y = two_complement_2_bin(shuffle(chunk.raw_mag.y));
  raw_mag->z = two_complement_2_bin(shuffle(chunk.raw_mag.z));

  raw_mag->x = get_adj_raw_mag(raw_mag->x, MPU6050_X_AXIS);
  raw_mag->y = get_adj_raw_mag(raw_mag->y, MPU6050_Y_AXIS);
  raw_mag->z = get_adj_raw_mag(raw_mag->z, MPU6050_Z_AXIS);

  return ESP_OK;

}


esp_err_t mpu9250_get_mag(mpu6050_dev_t *dev, ak8963_magnetometer_t *mag){
  CHECK_ARG(mag);

  ak8963_raw_magnetometer_t raw;
  CHECK(mpu9250_get_raw_mag(dev, &raw));

  mag->x = get_mag_value(raw->x);
  mag->y = get_mag_value(raw->y);
  mag->z = get_mag_value(raw->z);

  return ESP_OK;

}