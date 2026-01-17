#include "mpu6050.h"

// Creating a variable to hold whether an interrupt has been triggered
// 'volatile' meaning data can change outside of normal program flow
static volatile uint8_t sensor_data_ready = 0;

mp_obj_t mpu6050_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args){
    /**
     * This function initialises a new driver instance. It initialises the I2C bus and adds the IMU to it
     * Configures the interrupt pin for MPU6050
     * It then calls imu_setup to configure the MPU6050's config registers as required.
    */
    gpio_num_t scl_pin, sda_pin, int_pin;
    int8_t port;
    i2c_port_num_t i2c_port;
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t device_handle;
    esp_err_t err = ESP_ERR_INVALID_STATE;

    // Defining the allowed arguments, and setting default values for I2C port/address. Can be modified as keyword arguments
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_scl, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_sda, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_int, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_i2c_port, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = DEFAULT_I2C_PORT_NUM} },
    };

    // Checking arguments
    mp_arg_val_t parsed_args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, args, MP_ARRAY_SIZE(allowed_args), allowed_args, parsed_args);

    // Extracting arguments
    scl_pin = parsed_args[0].u_int;
    sda_pin = parsed_args[1].u_int;
    int_pin = parsed_args[2].u_int;
    port = parsed_args[3].u_int;

    // Ensuring I2C port number and pin numbers are valid
    if (!GPIO_IS_VALID_OUTPUT_GPIO(scl_pin) || !GPIO_IS_VALID_OUTPUT_GPIO(sda_pin) || !GPIO_IS_VALID_OUTPUT_GPIO(int_pin)){
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("Invalid SCL/SDA/interrupt pin number"));
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

    // Installing ISR handler
    gpio_install_isr_service(0);

    // Configuring interrupt pin
    gpio_config_t int_pin_config = {
        .pin_bit_mask = 1ULL<<int_pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };

    // Installing that config onto the GPIO pin
    gpio_config(&int_pin_config);

    // Registering ISR handler
    gpio_isr_handler_add(int_pin, interrupt_handler, NULL);

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

static void IRAM_ATTR interrupt_handler(void* arg){
    /**
     * Interrupt handler for the MPU6050 - just sets a flag to know that data is available
    */
    sensor_data_ready = 1;

    return;
}

static void write_to_register(mpu6050_obj_t* self, uint8_t reg, uint8_t data, char* error_text){
    /**
     * Utility to enable easy writes to MPU6050 config registers
     * Only writes single bytes of data
    */
    uint8_t write_data[2] = {reg, data};

    // Writing data
    err = i2c_master_transmit(self->device_handle, write_data, 2, 100);

    if (err != ESP_OK){
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT(error_text));
    }
}

static void read_from_register(mpu6050* self, uint8_t reg, uint8_t* output, char* error_text){
    /**
     * Utility to read 1 byte from a specific MPU6050 register
    */
    uint8_t write[1] = {reg};

    // Reading data
    err = i2c_master_transmit_receive(self->device_handle, write_whi, 1, output, 1, 20);

    if (err != ESP_OK){
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT(error_text));
    }
}

static void write_dmp_firmware(mpu6050_obj_t* self){
    /**
     * Function to write the firmware to the MPU6050's DMP
    */
    uint8_t bank, fw_byte, readback_fw_byte;
    uint16_t byte_offset, limit;

    // Iterating through firmware banks
    for (bank = 0; bank < NO_FW_BANKS - 1; bank++){
        // Writing firmware bank number
        write_to_register(self, DMP_CTRL_1, bank, "Failed to write DMP firmware bank number");

        // Setting up variables to write the bank data
        byte_offset = 0;

        if (bank == 11){
            limit = 246;
        }
        else {
            limit = 256;
        }

        // Writing bank data:
        while (byte_offset < limit){
            // Collecting firmware byte
            fw_byte = dmp_firwave_v612[bank*256 + byte_offset];

            // Writing firmware byte
            write_to_register(self, DMP_CTRL_2, byte_offset, "Couldn't write DMP firmware byte offset");
            write_to_register(self, DMP_CTRL_3, fw_byte, "Couldn't write DMP firmware byte");

            // Reading back the firmware byte
            write_to_register(self, DMP_CTRL_2, byte_offset, "Couldn't write DMP firmware byte offset");
            read_from_register(self, DMP_CTRL_3, &readback_fw_byte, "Couldn't read back DMP firmware byte");

            // Checking for errors: If none, then proceed
            if (fw_byte == readback_fw_byte){
                byte_offset ++;
            }

            // 1ms delay
            wait_micro_s(1000)
        }
    }
    return;
}

static void imu_setup(mpu6050_obj_t* self){
    /**
     * Writes to all the required MPU6050 registers to set up the module properly
    */
    uint8_t write_data[2];
    esp_err_t err;

    // Probing to check there's actually a device there
    err = i2c_master_probe(self->bus_handle, self->i2c_address, 100);

    if (err != ESP_OK){
        mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("MPU6050 device not found on I2C bus: %s"), esp_err_to_name(err));
    }

    // Setting DLPF to 42Hz (gyros) and 44Hz (accel)
    write_to_register(self, CONFIG_REG, 0x03, "Unable to write to sensor configuration register");

    // Setting sample rate to 200Hz
    write_to_register(self, SAMPLERATE_DIV_REG, 0x04, "Unable to write to sensor configuration register");

    // Enabling FIFO
    write_to_register(self, FIFO_EN_REG, 0x00, "Unable to write to sensor configuration register");

    // FIFO config
    write_to_register(self, USER_CTRL_REG, 0x04, "Unable to write to sensor configuration register");

    // Enabling Interrupts
    write_to_register(self, INT_EN_REG, 0x00, "Unable to write to sensor configuration register");
    write_to_register(self, INT_EN_REG, 0x02, "Unable to write to sensor configuration register");

    // Configuring interrupts
    write_to_register(self, INT_CONFIG_REG, 0x90, "Unable to write to sensor configuration register");

    // Writing DMP firmware
    write_dmp_firmware(self);

    // Writing DMP start byte
    write_to_register(self, DMP_FW_START_1, 0x04, "Unable to write to sensor configuration register");

    // Enable and reset both DMP and FIFO
    write_to_register(self, USER_CTRL_REG, 0xCC, "Unable to write to sensor configuration register");

    // Reading from the 'WHO AM I' register
    uint8_t read_whi;
    read_from_register(self, WHOAMI_REG, &read_whi, "Couldn't read MPU6050 'WHO AM I' register: Unknown error during setup");

    // Checking it's the expected response
    if (read_whi != 0x68){
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("I2C Connection Error - Bad return from 'MPU6050 WHO AM I' register"));
    }

    return;
}


static void wait_micro_s(uint32_t micro_s_delay){
    /**
     * Function to delay by a certain number of microseconds
    */
    uint64_t start = esp_timer_get_time();

    while (esp_timer_get_time() - start < micro_s_delay){}

    return;
}

static void log_func(const char* log_string){
    /**
     * Basic logging function - currently just prints to the REPL, but can be adapted to log to other places (e.g. log to a file) if needed
    */
    mp_printf(&mp_plat_print, "%s", log_string);
}

static void mparray_to_float(mp_obj_t array, float* output){
    /**
     * Converts from micropython array to an array of C floats
     * Only gets 4 items as this is used to convert the quaternion orientation [qw, qx, qy, qz] into C floats
    */
    size_t len;
    mp_obj_t *items;
    int i;

    mp_obj_get_array(array, &len, &items);

    if (len != 4){
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("Expected 4 values in list/tuple"));
    }

    for (i=0; i < len; i++){
        output[i] = mp_obj_get_float(items[i]);
    }
}

static mp_obj_t quat_to_euler(mp_obj_t self_in, mp_obj_t quat_mp){
    /**
     * Utility to convert from quaternion to Euler angles (degr)ees)
     * Uses aerospace/NASA standard conversion sequence - Z-Y-X
    */
    mpu6050_obj_t* self = MP_OBJ_TO_PTR(self_in);
    float quat[4], pitch, yaw, roll, arg;

    // Extracting quaternion
    mparray_to_float(quat_mp, quat);

    // Converting yaw/roll values
    yaw = atan2f(2*(quat[0]*quat[3] + quat[1]*quat[2]), 1-2*(quat[2]*quat[2]+quat[3]*quat[3]))*RAD_TO_DEG;
    roll = atan2f(2*(quat[0]*quat[1] + quat[2]*quat[3]), 1-2*(quat[1]*quat[1] + quat[2]*quat[2]))*RAD_TO_DEG;

    // Converting pitch values
    arg = 2*(quat[0]*quat[2] - quat[1]*quat[3]);

    // Bounding the value
    if (arg > 1.0f){
        arg = 1.0f;
    }
    else if (arg < -1.0f){
        arg = -1.0f;
    }

    pitch = asinf(arg);

    // Creating a micropython list to return
    mp_obj_t euler[3];

    euler[0] = mp_obj_new_float(pitch);
    euler[1] = mp_obj_new_float(roll);
    euler[2] = mp_obj_new_float(yaw);

    return mp_obj_new_list(3, euler);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mpu6050_quat_to_euler_obj, quat_to_euler);

mp_obj_t local_accel_get_world_accel(mp_obj_t self_in, mp_obj_t l_accel_mp, mp_obj_t quat_mp){
    /**
     * Converts acceleration in a local reference frame to a world reference frame using the quaternion orientation
    */
    float quat[4], l_accel[3], w_accel[3];

    // Extracting data arrays
    mparray_to_float(quat_mp, &quat);
    mparray_to_float(l_accel_mp, &l_accel);

    // Quaternion maths
    w_accel[0] = (quat[0]*quat[0]+quat[1]*quat[1]-quat[2]*quat[2]-quat[3]*quat[3])*l_accel[0] + 2*(quat[1]*quat[2]-quat[0]*quat[3])*l_accel[1] + 2*(quat[1]*quat[3]+quat[0]*quat[2])*l_accel[2];
    w_accel[1] = 2*(quat[1]*quat[2]+quat[0]*quat[3])*l_accel[0] + (quat[0]*quat[0]-quat[1]*quat[1]+quat[2]*quat[2]-quat[3]*quat[3])*l_accel[1] + 2*(quat[2]*quat[3]-quat[0]*quat[1])*l_accel[2];
    w_accel[2] = 2*(quat[1]*quat[3]-quat[0]*quat[2])*l_accel[0] + 2*(quat[2]*quat[3]+quat[0]*quat[1])*l_accel[1] + (quat[0]*quat[0]-quat[1]*quat[1]-quat[2]*quat[2]+quat[3]*quat[3])*l_accel[2];

    // Creating micropython list to return
    mp_obj_t ret_accel[3];

    ret_accel[0] = mp_obj_new_float(w_accel[0]);
    ret_accel[1] = mp_obj_new_float(w_accel[1]);
    ret_accel[2] = mp_obj_new_float(w_accel[2]);

    return mp_obj_new_list(3, ret_accel);
}
static MP_DEFINE_CONST_FUN_OBJ_3(mpu6050_lac_wac_obj, local_accel_get_world_accel);

mp_obj_t calibrate(mp_obj_t self_in, mp_obj_t time){}
static MP_DEFINE_CONST_FUN_OBJ_2(mpu6050_cal_obj, calibrate);

mp_obj_t get_data(mp_obj_t self_in){}
static MP_DEFINE_CONST_FUN_OBJ_2(mpu6050_getdata_obj, get_data);

/**
 * Code here exposes the module functions above to micropython as an object
*/

// Defining the functions that are exposed to micropython
static const mp_rom_map_elem_t mpu6050_locals_dict_table[] = {
    {MP_ROM_QSTR(MP_QSTR_quat_to_euler), MP_ROM_PTR(&mpu6050_quat_to_euler_obj)},
    {MP_ROM_QSTR(MP_QSTR_local_to_world_accel), MP_ROM_PTR(&mpu6050_lac_wac_obj)},
    {MP_ROM_QSTR(MP_QSTR_calibrate), MP_ROM_PTR(&mpu6050_cal_obj)},
    {MP_ROM_QSTR(MP_QSTR_get_data), MP_ROM_PTR(&mpu6050_getdata_obj)},
};
static MP_DEFINE_CONST_DICT(mpu6050_locals_dict, mpu6050_locals_dict_table);

// Overall module definition
MP_DEFINE_CONST_OBJ_TYPE(
    mpu6050_type,
    MP_QSTR_mpu6050,
    MP_TYPE_FLAG_NONE,
    make_new, mpu6050_make_new,
    locals_dict, &mpu6050_locals_dict
);

// Defining global constants
static const mp_rom_map_elem_t mpu6050_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__) , MP_ROM_QSTR(MP_QSTR_mpu6050) },
    { MP_ROM_QSTR(MP_QSTR_MPU6050), MP_ROM_PTR(&mpu6050_type) },
};
static MP_DEFINE_CONST_DICT(mpu6050_globals_table, mpu6050_module_globals_table);

// Creating module object
const mp_obj_module_t mpu6050_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t *)&mpu6050_globals_table,
};

MP_REGISTER_MODULE(MP_QSTR_mpu6050, mpu6050_module);
