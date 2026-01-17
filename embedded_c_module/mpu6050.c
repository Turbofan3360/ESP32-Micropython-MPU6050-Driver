#include "mpu6050.h"

mp_obj_t mpu6050_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args){
    /**
     * This function initialises a new driver instance. It initialises the I2C bus and adds the IMU to it
     * It then calls imu_setup to configure the MPU6050's config registers as required.
    */
    gpio_num_t scl_pin, sda_pin;
    int8_t port;
    i2c_port_num_t i2c_port;
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t device_handle;
    esp_err_t err = ESP_ERR_INVALID_STATE;

    // Defining the allowed arguments, and setting default values for I2C port/address. Can be modified as keyword arguments
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_scl, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_sda, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_i2c_port, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = DEFAULT_I2C_PORT_NUM} },
    };

    // Checking arguments
    mp_arg_val_t parsed_args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, args, MP_ARRAY_SIZE(allowed_args), allowed_args, parsed_args);

    // Extracting arguments
    scl_pin = parsed_args[0].u_int;
    sda_pin = parsed_args[1].u_int;
    port = parsed_args[2].u_int;

    // Ensuring I2C port number and pin numbers are valid
    if (!GPIO_IS_VALID_OUTPUT_GPIO(scl_pin) || !GPIO_IS_VALID_OUTPUT_GPIO(sda_pin)){
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("Invalid SCL or SDA pin number"));
    }

    if ((port < -1) || (port > 1)){
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("Invalid I2C port number"));
    }

    // If port is not set to autoselect:
    if (port != -1){
        // Trying to pull handle for the bus (if it's already been initialized)
        err = i2c_master_get_bus_handle(port, &bus_handle);
    }

    // If there's no already initialized bus handle or port is set to autoselect, then create a bus:
    if (err == ESP_ERR_INVALID_STATE){
        // Setting I2C port value
        if (port == -1){
            i2c_port = -1;
        }
        else if (port == 0){
            i2c_port = I2C_NUM_0;
        }
        else if (port == 1){
            i2c_port = I2C_NUM_1;
        }

        // Configuring ESP-IDF I2C bus object
        i2c_master_bus_config_t i2c_mst_config = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = i2c_port,
            .scl_io_num = scl_pin,
            .sda_io_num = sda_pin,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };

        // Creating the bus
        err = i2c_new_master_bus(&i2c_mst_config, &bus_handle);

        if (err != ESP_OK){
            mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("Error initialising I2C bus: %s"), esp_err_to_name(err));
        }
    }

    // Adding the MPU6050 slave device to the bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_I2C_ADDRESS,
        .scl_speed_hz = 400000,
    };

    // Installing this to the bus
    err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &device_handle);

    if (err != ESP_OK){
        mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("Error adding device to I2C bus: %s"), esp_err_to_name(err));
    }

    // Creating the "self" object for this driver
    mpu6050_obj_t* self = m_new_obj(mpu6050_obj_t);

    // Initialising the required data in the "self" object
    self->base.type = &mpu6050_type;
    self->bus_handle = bus_handle;
    self->device_handle = device_handle;
    self->i2c_address = MPU6050_I2C_ADDRESS;



    // 1ms delay to ensure the chip powers up properly
    wait_micro_s(1000);

    imu_setup(self);

    return MP_OBJ_FROM_PTR(self);
}

static void imu_setup(mpu6050_obj_t *self){
}


static void wait_micro_s(uint32_t micro_s_delay){
    /**
     * Function to delay by a certain number of microseconds
    */
    uint64_t start = esp_timer_get_time();

    while (esp_timer_get_time() - start < micro_s_delay){}

    return;
}

static void log_func(const char *log_string){
    /**
     * Basic logging function - currently just prints to the REPL, but can be adapted to log to other places (e.g. log to a file) if needed
    */
    mp_printf(&mp_plat_print, "%s", log_string);
}
