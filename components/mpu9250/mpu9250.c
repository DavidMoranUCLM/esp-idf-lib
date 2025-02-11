#include "mpu6050.h"
#include "mpu6050_regs.h"
#include "mpu9250.h"
#include "ak8963_regs.h"
#include <math.h>
#include "string.h"
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

static struct{
    float A[3][3]; //Rotation and deformation
    float b[3]; //Bias
} mag_cal_values;

static struct {
    float b[3]; //Bias
    float s[3]; //Scale
}acc_cal_values;

static struct{
    float b[3]; //Bias
    float s[3]; //Scale
}gyro_cal_values;




static void mat_vec_prod(int m, int n, float A[m][n], float x[n], float y[m]) {
    for (int i = 0; i < m; i++) {
        y[i] = 0;
        for (int j = 0; j < n; j++) {
            y[i] += A[i][j] * x[j];
        }
    }
}

inline static float get_mag_value(float raw)
{
    return raw * mag_res;
}

inline static float get_adj_raw_mag(int16_t raw, mpu6050_axis_t axis)
{
    return (((((float)mag_adj[axis] - 128)) / 256) + 1) * (float)raw;
}

inline static int8_t two_complement_is_neg(uint16_t word)
{
    return (~word & TWO_COMPLEMENT_SIGN_MASK) >> 15;
}

inline static int16_t two_complement_2_bin(uint16_t word)
{
    // uint16_t abs_word = (word << 1) >> 1
    if (two_complement_is_neg(word))
    {
        return -((~word + 1) & 0xFFFF);
    }
    else
    {
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
    ESP_LOGD(TAG, "I2C Write buffer: %02x", buf);

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

static esp_err_t mpu9250_set_mag_mode(mpu9250_dev_t *dev, ak8963_mode_t mode)
{
    if (mode != AK8963_POWER_DOWN)
    {
        CHECK(write_reg_bits(dev, AK8963_CNTL1_ADDR, 0, AK8963_CNTL1_MODE_MASK, ak8963_mode_values[AK8963_POWER_DOWN]));
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    esp_err_t err = write_reg_bits(dev, AK8963_CNTL1_ADDR, 0, AK8963_CNTL1_MODE_MASK, ak8963_mode_values[mode]);
    if (err == ESP_OK)
        vTaskDelay(pdMS_TO_TICKS(1));

    return err;
}

static esp_err_t mpu9250_get_mag_mode(mpu9250_dev_t *dev, ak8963_mode_t *mode)
{
    uint8_t buf;
    CHECK(read_reg_bits(dev, AK8963_CNTL1_ADDR, 0, AK8963_CNTL1_MODE_MASK, &buf));

    for (uint8_t i = 0; i < AK8963_MAX_MODES; i++)
    {
        if (ak8963_mode_values[i] == buf)
        {
            *mode = i;
            return ESP_OK;
        }
    }

    return ESP_ERR_NOT_FOUND;
}

static esp_err_t mpu9250_get_mag_adj_values(mpu9250_dev_t *dev)
{
    CHECK(mpu9250_set_mag_mode(dev, AK8963_FUSE_ACCESS));
    CHECK(read_reg_bits(dev, AK8963_ASAX_ADDR, 0, 0xF, &mag_adj[0]));
    CHECK(read_reg_bits(dev, AK8963_ASAY_ADDR, 0, 0xF, &mag_adj[1]));
    CHECK(read_reg_bits(dev, AK8963_ASAZ_ADDR, 0, 0xF, &mag_adj[2]));
    CHECK(mpu9250_set_mag_mode(dev, AK8963_POWER_DOWN));

    return ESP_OK;
}

static esp_err_t mpu9250_set_mag_out_bits(mpu9250_dev_t *dev, bool mode)
{
    return write_reg_bits(dev, AK8963_CNTL1_ADDR, AK8963_CNTL1_BIT_OFFSET, AK8963_CNTL1_BIT_MASK, mode);
}

static esp_err_t mpu9250_check_ak8963(mpu9250_dev_t *dev)
{
    uint8_t buf = 0;

    CHECK(read_reg(dev, AK8963_WIA_ADDR, &buf));
    if (buf != AK8963_WIA_VALUE)
    {
        ESP_LOGW(TAG, "AK8963 WIA val expected %02x, read %02x", AK8963_WIA_VALUE, buf);
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_LOGI(TAG, "AK8963 WIA value correct");

    ak8963_mode_t read_mode;
    ESP_LOGI(TAG, "AK8963 Trying mode change");
    for (uint8_t mode = 0; mode < AK8963_MAX_MODES; mode++)
    {
        CHECK(mpu9250_set_mag_mode(dev, mode));
        CHECK(mpu9250_get_mag_mode(dev, &read_mode));

        if (read_mode != mode)
        {
            ESP_LOGW(TAG, "AK8963 Mode change error. Expected %02x, read %02x", mode, read_mode);
        }
        ESP_LOGI(TAG, "AK8963 Mode change %x success", mode);
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    ESP_LOGI(TAG, "AK8963 Power Down Mode");
    CHECK(mpu9250_set_mag_mode(dev, AK8963_POWER_DOWN));
    vTaskDelay(pdMS_TO_TICKS(1));
    ESP_LOGI(TAG, "AK8963 Bit output change");
    CHECK(mpu9250_set_mag_out_bits(dev, 1));
    vTaskDelay(pdMS_TO_TICKS(1));
    ESP_LOGI(TAG, "AK8963 Measurement mode");
    CHECK(mpu9250_set_mag_mode(dev, AK8963_CONTINUOUS_MEASUREMENT_2));
    vTaskDelay(pdMS_TO_TICKS(1));

    ak8963_read_chunk_t chunk = { 0 };

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, AK8963_ST1_ADDR, &chunk, sizeof(chunk)));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    esp_log_buffer_hex_internal(TAG, &chunk, sizeof(chunk), ESP_LOG_INFO);

    return ESP_OK;
}

static esp_err_t apply_mag_cal(ak8963_magnetometer_t *mag){
    ak8963_magnetometer_t mag0;

    mag->x -= mag_cal_values.b[MPU6050_X_AXIS];
    mag->y -= mag_cal_values.b[MPU6050_Y_AXIS];
    mag->z -= mag_cal_values.b[MPU6050_Z_AXIS];

    memcpy(&mag0, mag, sizeof(*mag));
    mat_vec_prod(3,3,mag_cal_values.A,&mag0,mag);
    

    return ESP_OK;
}

esp_err_t mpu9250_init_desc(mpu9250_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK(mpu6050_init_desc(&dev->mpu6050_dev, addr, port, sda_gpio, scl_gpio));

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
    CHECK(mpu6050_free_desc(&dev->mpu6050_dev));
    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t mpu9250_init(mpu9250_dev_t *dev)
{
    CHECK(mpu6050_init(&dev->mpu6050_dev));
    CHECK(mpu6050_set_i2c_master_mode_enabled(&dev->mpu6050_dev, 0));
    CHECK(mpu6050_set_i2c_bypass_enabled(&dev->mpu6050_dev, 1));
    CHECK(mpu9250_get_mag_adj_values(dev));
    CHECK(mpu9250_set_mag_out_bits(dev, 1));
    CHECK(mpu9250_set_mag_mode(dev, AK8963_CONTINUOUS_MEASUREMENT_2));

    for(uint8_t i=0;i<3;i++){
        for(uint8_t j=0;j<3;j++){
            if(i==j){
                mag_cal_values.A[i][j] = 1;
            }
            else{
                mag_cal_values.A[i][j] = 0;
            }
        }
    }
    

    return ESP_OK;
}

esp_err_t mpu9250_set_mag_cal(float A[3][3], float b[3]){
    memcpy(mag_cal_values.A, A, sizeof(mag_cal_values.A));
    memcpy(mag_cal_values.b, b, sizeof(mag_cal_values.b));
    return ESP_OK;
}

esp_err_t mpu9250_set_accel_cal(float s[3], float b[3]){
    memcpy(acc_cal_values.b, b, sizeof(acc_cal_values.b));
    memcpy(acc_cal_values.s, s, sizeof(acc_cal_values.s));
    return ESP_OK;
}

esp_err_t mpu9250_set_gyro_cal(float s[3], float b[3]){
    memcpy(gyro_cal_values.b, b, sizeof(gyro_cal_values.b));
    memcpy(gyro_cal_values.s, s, sizeof(gyro_cal_values.s));
    return ESP_OK;
}


esp_err_t mpu9250_get_raw_mag(mpu9250_dev_t *dev, ak8963_raw_magnetometer_t *raw_mag)
{
    CHECK_ARG(dev && raw_mag);

    ak8963_read_chunk_t chunk = { 0 };

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, i2c_dev_read_reg(&dev->i2c_dev, AK8963_ST1_ADDR, &chunk, sizeof(chunk)));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    ESP_LOGD(TAG, "xhunk size: %d", sizeof(chunk));
    esp_log_buffer_hex_internal(TAG, &chunk, sizeof(chunk), ESP_LOG_DEBUG);

    raw_mag->x = two_complement_2_bin(chunk.raw_mag.x);
    raw_mag->y = two_complement_2_bin(chunk.raw_mag.y);
    raw_mag->z = two_complement_2_bin(chunk.raw_mag.z);

    ESP_LOGD(TAG, "mag raw x val 0x%04x", raw_mag->x);
    ESP_LOGD(TAG, "mag raw y val 0x%04x", raw_mag->y);
    ESP_LOGD(TAG, "mag raw z val 0x%04x", raw_mag->z);

    raw_mag->x = get_adj_raw_mag(raw_mag->x, MPU6050_X_AXIS);
    raw_mag->y = get_adj_raw_mag(raw_mag->y, MPU6050_Y_AXIS);
    raw_mag->z = get_adj_raw_mag(raw_mag->z, MPU6050_Z_AXIS);

    return ESP_OK;
}

esp_err_t mpu9250_get_mag(mpu9250_dev_t *dev, ak8963_magnetometer_t *mag)
{
    CHECK_ARG(mag);

    ak8963_raw_magnetometer_t raw;
    CHECK(mpu9250_get_raw_mag(dev, &raw));

    mag->x = get_mag_value(raw.x);
    mag->y = get_mag_value(raw.y);
    mag->z = get_mag_value(raw.z);

    apply_mag_cal(mag);

    return ESP_OK;
}

esp_err_t mpu9250_get_motion(mpu9250_dev_t *dev, mpu6050_acceleration_t *data_accel, mpu6050_rotation_t *data_gyro, ak8963_magnetometer_t *data_mag){
    CHECK(mpu6050_get_acceleration(&dev->mpu6050_dev, data_accel));
    CHECK(mpu6050_get_rotation(&dev->mpu6050_dev, data_gyro));
    CHECK(mpu9250_get_mag(dev, data_mag));

    return ESP_OK;
}


esp_err_t mpu9250_tests(mpu9250_dev_t *dev,uint8_t addr, gpio_num_t sda, gpio_num_t scl)
{
    ESP_LOGI(TAG, "MPU9250 desc init try 0");
    ESP_ERROR_CHECK(mpu9250_init_desc(dev, addr, 0, sda, scl));
    ESP_LOGI(TAG, "MPU9250 desc init success");

    while (1)
    {
        esp_err_t res = i2c_dev_probe(&dev->mpu6050_dev.i2c_dev, I2C_DEV_WRITE);
        if (res != ESP_OK)
        {
            ESP_LOGE(TAG, "MPU60x0 not found %s", esp_err_to_name(res));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        ESP_LOGI(TAG, "Found MPU60x0 device");
        res = i2c_dev_probe(&dev->i2c_dev, I2C_DEV_WRITE);
        if (res != ESP_OK)
        {
            ESP_LOGE(TAG, "AK8963 not found: %s", esp_err_to_name(res));
            vTaskDelay(pdMS_TO_TICKS(1000));
            break;
        }
        ESP_LOGI(TAG, "Found AK8963 device");
        break;
    }

    ESP_LOGI(TAG, "MPU9250 init try...");
    ESP_ERROR_CHECK(mpu9250_init(dev));
    ESP_LOGI(TAG, "MPU9250 init Success\n");

    if(mpu9250_check_ak8963(dev) != ESP_OK){
        mpu9250_free_desc(dev);
        return ESP_FAIL;
    }

    // Two Complement Tests

    int16_t res = two_complement_2_bin(0xFFFF);

    if (res != -1)
    {
        ESP_LOGW(TAG, "Two complement to bin from 0xFFFF expected: -1, was %d", res);
    }
    res = two_complement_2_bin(0x0000);
    if (res != 0)
    {
        ESP_LOGW(TAG, "Two complement to bin from 0x000 expected: 0, was %d", res);
    }
    res = two_complement_2_bin(0x0001);
    if (res != 1)
    {
        ESP_LOGW(TAG, "Two complement to bin from 0x0x0001 expected: 1, was %d", res);
    }
    res = two_complement_2_bin(0x7FF8);
    if (res != 32760)
    {
        ESP_LOGW(TAG, "Two complement to bin from 0x7FF8 expected: 32760, was %d", res);
    }
    res = two_complement_2_bin(0x8008);
    if (res != -32760)
    {
        ESP_LOGW(TAG, "Two complement to bin from 0x8008 expected: -32760, was %d", res);
    }

    ak8963_magnetometer_t mag = { 0 };

    // for (uint8_t i = 0; i < 20; i++)
    // {
    //     ESP_ERROR_CHECK(mpu9250_get_mag(&dev, &mag));
    //     ESP_LOGI(TAG, "Magnetic:     x=%.4f   y=%.4f   z=%.4f", mag.x, mag.y, mag.z);
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }

    

    return ESP_OK;
}