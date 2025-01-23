idf_component_register(
    SRCS "mpu9250.c" "mpu9050.c"
    INCLUDE_DIRS .
    REQUIRES i2cdev log esp_idf_lib_helpers mpu6050
)
