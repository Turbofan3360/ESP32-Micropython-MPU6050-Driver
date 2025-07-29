# Micropythony MPU-6050 DMP Driver for ESP32 #

### The Code: ###

This is a driver designed to utilise the MPU6050 IMU's onboard Digital Motion Processor (DMP). The code configures the module's processor to the required settings, and loads the firmware (either the v2 or v6.12, both files are included). The code then receives and decodes the quaternion orientation data from the DMP, as well as the acceleration. The code then uses a quaternion rotation to remove the effect of gravity from the accelerometer readings, as well as rotate the accelerometer vector into the world reference frame. The code then returns accelerometer readings, with gravity subtracted, in both the local reference frame and world reference frame.

The quat_to_euler(quaternion) method uses the aerospace standard Z-Y-X conversion format to convert from quaternion orientation to Euler angles.

### Implementation Example: ###

```python3
import mpu6050

module = mpu6050.MPU6050(46, 3) # SCL and SDA pins for the module
module.dmpsetup(1) # INT pin for the module
module.calibrate(10) # The length of time, in seconds, that the code runs raw calibration for

quaternion, local_accel, world_accel = module.imutrack() # Quaternion in [qw, qx, qy, qz] format, accel in format [ax, ay, az]

euler_angles = module.quat_to_euler
```

### References: ###

 - <https://datasheets.b-cdn.net/files/MPU-6050-InvenSense-datasheet-8859467.pdf>
 - <https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf>
 - <https://www.scribd.com/document/661093093/Application-Note-Programming-Sequence-for-DMP-Hardware-Functions-v12#page=1>
 - <https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050_6Axis_MotionApps612.cpp#L454>