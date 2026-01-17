#ifndef MPU6050_H
#define MPU6050_H

#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "py/runtime.h"
#include "py/obj.h"
#include "py/objstr.h"
#include "py/mphal.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"

// Register addresses

// Constant definitions
#define DEFAULT_I2C_PORT_NUM -1
#define MPU6050_I2C_ADDRESS 0x68

// Object definition
typedef struct {
    mp_obj_base_t base;

	uint8_t i2c_address;
	i2c_master_bus_handle_t bus_handle;
	i2c_master_dev_handle_t device_handle;
} mpu6050_obj_t;

// Function declarations
static void imu_setup(mpu6050_obj_t *self);
static void log_func(const char *log_string);
static void wait_micro_s(uint32_t micro_s_delay);

extern const mp_obj_type_t mpu6050_type;

#endif
