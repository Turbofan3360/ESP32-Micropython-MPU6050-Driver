# Micropythony MPU-6050 DMP Driver for ESP32 #

### The Code: ###

This is a driver designed to utilise the MPU6050 IMU's onboard Digital Motion Processor (DMP). The code configures the module's processor to the required settings, and loads the firmware (either the v2 or v6.12, both files are included). The code then receives and decodes the quaternion orientation data from the DMP, as well as the acceleration. The code then uses a quaternion rotation to remove the effect of gravity from the accelerometer readings, as well as rotate the accelerometer vector into the world reference frame. The code then returns accelerometer readings, with gravity subtracted, in both the local reference frame and world reference frame.

The quat_to_euler(quaternion) method uses the aerospace standard Z-Y-X conversion format to convert from quaternion orientation to Euler angles.

In the directory embedded_c_module, you will find the .c, .h, and .cmake files required to compile this QMC58833P driver into your micropython firmware. This has the full functionality of the MicroPython driver. See below for details on how to compile this into your micropython board's firmware.

### Micropython Example: ###

```python3
import mpu6050

module = mpu6050.MPU6050(46, 3) # SCL and SDA pins for the module
module.dmpsetup(1) # INT pin for the module
module.calibrate(10) # The length of time, in seconds, that the code runs raw calibration for

quaternion, local_accel, world_accel = module.imutrack() # Quaternion in [qw, qx, qy, qz] format, accel in format [ax, ay, az]

euler_angles = module.quat_to_euler(quaternion)
```

### Embedded C Module Example: ###

```python3
import mpu6050

scl_pin = 1
sda_pin = 2
# Port can be either 0 or 1, or -1 to automatically select. Default is -1
port = 0

imu = mpu6050.MPU6050(scl_pin, sda_pin, i2c_port=port) # May take 10-15 seconds as it needs to load firmware onto the MPU6050's DMP
imu.calibrate(5) # Parameter is length of time coarse calibration is run for
quaternion, local_accel = imu.get_data() # Local acceleration has gravity subtracted from readings
euler_angles = imu.quat_to_euler(quaternion)
world_accel = imu.local_to_world_accel(local_accel, quaternion) # Rotates local acceleration values into whatever reference frame defined by the quaternion orientation
```
SCL/SDA pins are required, i2c_port is an optional parameter.

### Compiling the module into firmware: ###

To do this, you will need:
 - ESP-IDF cloned from github
 - Micropython cloned from github

1. Enter your esp-idf directory, and run ./install.sh (only needs to be run the first time)
2. Enter your esp-idf directory and run . ./export.sh (needs to be run every new terminal session)
3. Download the files from embedded_c_module and place them in a directory of your choosing
4. Enter your directory ~/micropython/ports/esp32 (can be replaced with whichever micropython board you are using if the code is suitably adapted)
5. Run the make command, specifying USER_C_MODULES=/path/to/MPU6050_DMP/embedded_c_module/micropython.cmake (replace with your file path)

For me, with an ESP32-S3 that has octal SPIRAM, the full make command is:
```
make BOARD=ESP32_GENERIC_S3 BOARD_VARIANT=SPIRAM_OCT USER_C_MODULES=/path/to/MPU6050_DMP/embedded_c_module/micropython.cmake
```

### References: ###

 - <https://datasheets.b-cdn.net/files/MPU-6050-InvenSense-datasheet-8859467.pdf>
 - <https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf>
 - <https://www.scribd.com/document/661093093/Application-Note-Programming-Sequence-for-DMP-Hardware-Functions-v12#page=1>
 - <https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050_6Axis_MotionApps612.cpp#L454>