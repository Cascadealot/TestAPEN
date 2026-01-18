# ICM-20948 IMU Driver Component

idf_component_register(
    SRCS
        "icm20948.c"
        "madgwick.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        driver
        freertos
        param_store
)
