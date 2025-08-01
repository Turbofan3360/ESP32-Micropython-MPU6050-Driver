from machine import SoftI2C, Pin
from math import asin, atan2
import time, struct

class I2CConnectionError(Exception):
    pass

class MPU6050:
    def __init__(self, scl, sda):
        self.module = SoftI2C(scl=Pin(scl), sda=Pin(sda), freq=400000)
        
        self.registers = {
            "accel_config" : 0x1C,
            "gyro_config" : 0x1B,
            "fifo" : 0x74,
            
            "pwr_mgmnt" : 0x6B,
            "pwr_mgmnt_2" : 0x6C,
            "config" : 0x1A,
            "user_ctrl" : 0x6A,
            "smplrt_div" : 0x19,
            "WHO_AM_I" : 0x75,
            
            "dmp_ctrl_1" : 0x6D,
            "dmp_ctrl_2" : 0x6E,
            "dmp_ctrl_3" : 0x6F,
            "fifo_en" : 0x23,
            "int_enable" : 0x38,
            "int_pin_config" : 0x37,
            "int_status" : 0x3A,
            
            "dmp_firmware_start_1" : 0x70,
            "dmp_firmware_start_2" : 0x71
            }
        
        FIFO_REG = const(0x74)
        IMUADDRESS = const(0x68)

        self.dmp_firmware_v20 = [
            # Bank 0
            0xFB, 0x00, 0x00, 0x3E, 0x00, 0x0B, 0x00, 0x36, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00,
            0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0xFA, 0x80, 0x00, 0x0B, 0x12, 0x82, 0x00, 0x01,
            0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x28, 0x00, 0x00, 0xFF, 0xFF, 0x45, 0x81, 0xFF, 0xFF, 0xFA, 0x72, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x7F, 0xFF, 0xFF, 0xFE, 0x80, 0x01,
            0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x02, 0xCB, 0x47, 0xA2, 0x20, 0x00, 0x00, 0x00,
            0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
            0x41, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x2A, 0x00, 0x00, 0x16, 0x55, 0x00, 0x00, 0x21, 0x82,
            0xFD, 0x87, 0x26, 0x50, 0xFD, 0x80, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x05, 0x80, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00,
            0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x6F, 0x00, 0x02, 0x65, 0x32, 0x00, 0x00, 0x5E, 0xC0,
            0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0xFB, 0x8C, 0x6F, 0x5D, 0xFD, 0x5D, 0x08, 0xD9, 0x00, 0x7C, 0x73, 0x3B, 0x00, 0x6C, 0x12, 0xCC,
            0x32, 0x00, 0x13, 0x9D, 0x32, 0x00, 0xD0, 0xD6, 0x32, 0x00, 0x08, 0x00, 0x40, 0x00, 0x01, 0xF4,
            0xFF, 0xE6, 0x80, 0x79, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0xD6, 0x00, 0x00, 0x27, 0x10,
            # Bank 1
            0xFB, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
            0x00, 0x00, 0xFA, 0x36, 0xFF, 0xBC, 0x30, 0x8E, 0x00, 0x05, 0xFB, 0xF0, 0xFF, 0xD9, 0x5B, 0xC8,
            0xFF, 0xD0, 0x9A, 0xBE, 0x00, 0x00, 0x10, 0xA9, 0xFF, 0xF4, 0x1E, 0xB2, 0x00, 0xCE, 0xBB, 0xF7,
            0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x02, 0x00, 0x02, 0x02, 0x00, 0x00, 0x0C,
            0xFF, 0xC2, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0xCF, 0x80, 0x00, 0x40, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x09, 0x23, 0xA1, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x03, 0x3F, 0x68, 0xB6, 0x79, 0x35, 0x28, 0xBC, 0xC6, 0x7E, 0xD1, 0x6C,
            0x80, 0x00, 0xFF, 0xFF, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x6A, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0, 0x00, 0x00, 0x00, 0x30,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00,
            0x00, 0x00, 0x25, 0x4D, 0x00, 0x2F, 0x70, 0x6D, 0x00, 0x00, 0x05, 0xAE, 0x00, 0x0C, 0x02, 0xD0,
            # Bank 2
            0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x01, 0x00, 0x00, 0x44, 0x00, 0x01, 0x00, 0x05, 0x8B, 0xC1, 0x00, 0x00, 0x01, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x00, 0x00, 0x54, 0x00, 0x00, 0xFF, 0xEF, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            # Bank 3
            0xD8, 0xDC, 0xBA, 0xA2, 0xF1, 0xDE, 0xB2, 0xB8, 0xB4, 0xA8, 0x81, 0x91, 0xF7, 0x4A, 0x90, 0x7F,
            0x91, 0x6A, 0xF3, 0xF9, 0xDB, 0xA8, 0xF9, 0xB0, 0xBA, 0xA0, 0x80, 0xF2, 0xCE, 0x81, 0xF3, 0xC2,
            0xF1, 0xC1, 0xF2, 0xC3, 0xF3, 0xCC, 0xA2, 0xB2, 0x80, 0xF1, 0xC6, 0xD8, 0x80, 0xBA, 0xA7, 0xDF,
            0xDF, 0xDF, 0xF2, 0xA7, 0xC3, 0xCB, 0xC5, 0xB6, 0xF0, 0x87, 0xA2, 0x94, 0x24, 0x48, 0x70, 0x3C,
            0x95, 0x40, 0x68, 0x34, 0x58, 0x9B, 0x78, 0xA2, 0xF1, 0x83, 0x92, 0x2D, 0x55, 0x7D, 0xD8, 0xB1,
            0xB4, 0xB8, 0xA1, 0xD0, 0x91, 0x80, 0xF2, 0x70, 0xF3, 0x70, 0xF2, 0x7C, 0x80, 0xA8, 0xF1, 0x01,
            0xB0, 0x98, 0x87, 0xD9, 0x43, 0xD8, 0x86, 0xC9, 0x88, 0xBA, 0xA1, 0xF2, 0x0E, 0xB8, 0x97, 0x80,
            0xF1, 0xA9, 0xDF, 0xDF, 0xDF, 0xAA, 0xDF, 0xDF, 0xDF, 0xF2, 0xAA, 0x4C, 0xCD, 0x6C, 0xA9, 0x0C,
            0xC9, 0x2C, 0x97, 0x97, 0x97, 0x97, 0xF1, 0xA9, 0x89, 0x26, 0x46, 0x66, 0xB0, 0xB4, 0xBA, 0x80,
            0xAC, 0xDE, 0xF2, 0xCA, 0xF1, 0xB2, 0x8C, 0x02, 0xA9, 0xB6, 0x98, 0x00, 0x89, 0x0E, 0x16, 0x1E,
            0xB8, 0xA9, 0xB4, 0x99, 0x2C, 0x54, 0x7C, 0xB0, 0x8A, 0xA8, 0x96, 0x36, 0x56, 0x76, 0xF1, 0xB9,
            0xAF, 0xB4, 0xB0, 0x83, 0xC0, 0xB8, 0xA8, 0x97, 0x11, 0xB1, 0x8F, 0x98, 0xB9, 0xAF, 0xF0, 0x24,
            0x08, 0x44, 0x10, 0x64, 0x18, 0xF1, 0xA3, 0x29, 0x55, 0x7D, 0xAF, 0x83, 0xB5, 0x93, 0xAF, 0xF0,
            0x00, 0x28, 0x50, 0xF1, 0xA3, 0x86, 0x9F, 0x61, 0xA6, 0xDA, 0xDE, 0xDF, 0xD9, 0xFA, 0xA3, 0x86,
            0x96, 0xDB, 0x31, 0xA6, 0xD9, 0xF8, 0xDF, 0xBA, 0xA6, 0x8F, 0xC2, 0xC5, 0xC7, 0xB2, 0x8C, 0xC1,
            0xB8, 0xA2, 0xDF, 0xDF, 0xDF, 0xA3, 0xDF, 0xDF, 0xDF, 0xD8, 0xD8, 0xF1, 0xB8, 0xA8, 0xB2, 0x86,
            # Bank 4
            0xB4, 0x98, 0x0D, 0x35, 0x5D, 0xB8, 0xAA, 0x98, 0xB0, 0x87, 0x2D, 0x35, 0x3D, 0xB2, 0xB6, 0xBA,
            0xAF, 0x8C, 0x96, 0x19, 0x8F, 0x9F, 0xA7, 0x0E, 0x16, 0x1E, 0xB4, 0x9A, 0xB8, 0xAA, 0x87, 0x2C,
            0x54, 0x7C, 0xB9, 0xA3, 0xDE, 0xDF, 0xDF, 0xA3, 0xB1, 0x80, 0xF2, 0xC4, 0xCD, 0xC9, 0xF1, 0xB8,
            0xA9, 0xB4, 0x99, 0x83, 0x0D, 0x35, 0x5D, 0x89, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0xB5, 0x93, 0xA3,
            0x0E, 0x16, 0x1E, 0xA9, 0x2C, 0x54, 0x7C, 0xB8, 0xB4, 0xB0, 0xF1, 0x97, 0x83, 0xA8, 0x11, 0x84,
            0xA5, 0x09, 0x98, 0xA3, 0x83, 0xF0, 0xDA, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xD8, 0xF1, 0xA5,
            0x29, 0x55, 0x7D, 0xA5, 0x85, 0x95, 0x02, 0x1A, 0x2E, 0x3A, 0x56, 0x5A, 0x40, 0x48, 0xF9, 0xF3,
            0xA3, 0xD9, 0xF8, 0xF0, 0x98, 0x83, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0x97, 0x82, 0xA8, 0xF1,
            0x11, 0xF0, 0x98, 0xA2, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xDA, 0xF3, 0xDE, 0xD8, 0x83, 0xA5,
            0x94, 0x01, 0xD9, 0xA3, 0x02, 0xF1, 0xA2, 0xC3, 0xC5, 0xC7, 0xD8, 0xF1, 0x84, 0x92, 0xA2, 0x4D,
            0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
            0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0x93, 0xA3, 0x4D,
            0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
            0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0xA8, 0x8A, 0x9A,
            0xF0, 0x28, 0x50, 0x78, 0x9E, 0xF3, 0x88, 0x18, 0xF1, 0x9F, 0x1D, 0x98, 0xA8, 0xD9, 0x08, 0xD8,
            0xC8, 0x9F, 0x12, 0x9E, 0xF3, 0x15, 0xA8, 0xDA, 0x12, 0x10, 0xD8, 0xF1, 0xAF, 0xC8, 0x97, 0x87,
            # Bank 5
            0x34, 0xB5, 0xB9, 0x94, 0xA4, 0x21, 0xF3, 0xD9, 0x22, 0xD8, 0xF2, 0x2D, 0xF3, 0xD9, 0x2A, 0xD8,
            0xF2, 0x35, 0xF3, 0xD9, 0x32, 0xD8, 0x81, 0xA4, 0x60, 0x60, 0x61, 0xD9, 0x61, 0xD8, 0x6C, 0x68,
            0x69, 0xD9, 0x69, 0xD8, 0x74, 0x70, 0x71, 0xD9, 0x71, 0xD8, 0xB1, 0xA3, 0x84, 0x19, 0x3D, 0x5D,
            0xA3, 0x83, 0x1A, 0x3E, 0x5E, 0x93, 0x10, 0x30, 0x81, 0x10, 0x11, 0xB8, 0xB0, 0xAF, 0x8F, 0x94,
            0xF2, 0xDA, 0x3E, 0xD8, 0xB4, 0x9A, 0xA8, 0x87, 0x29, 0xDA, 0xF8, 0xD8, 0x87, 0x9A, 0x35, 0xDA,
            0xF8, 0xD8, 0x87, 0x9A, 0x3D, 0xDA, 0xF8, 0xD8, 0xB1, 0xB9, 0xA4, 0x98, 0x85, 0x02, 0x2E, 0x56,
            0xA5, 0x81, 0x00, 0x0C, 0x14, 0xA3, 0x97, 0xB0, 0x8A, 0xF1, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9,
            0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x84, 0x0D, 0xDA, 0x0E, 0xD8, 0xA3, 0x29, 0x83, 0xDA,
            0x2C, 0x0E, 0xD8, 0xA3, 0x84, 0x49, 0x83, 0xDA, 0x2C, 0x4C, 0x0E, 0xD8, 0xB8, 0xB0, 0xA8, 0x8A,
            0x9A, 0xF5, 0x20, 0xAA, 0xDA, 0xDF, 0xD8, 0xA8, 0x40, 0xAA, 0xD0, 0xDA, 0xDE, 0xD8, 0xA8, 0x60,
            0xAA, 0xDA, 0xD0, 0xDF, 0xD8, 0xF1, 0x97, 0x86, 0xA8, 0x31, 0x9B, 0x06, 0x99, 0x07, 0xAB, 0x97,
            0x28, 0x88, 0x9B, 0xF0, 0x0C, 0x20, 0x14, 0x40, 0xB8, 0xB0, 0xB4, 0xA8, 0x8C, 0x9C, 0xF0, 0x04,
            0x28, 0x51, 0x79, 0x1D, 0x30, 0x14, 0x38, 0xB2, 0x82, 0xAB, 0xD0, 0x98, 0x2C, 0x50, 0x50, 0x78,
            0x78, 0x9B, 0xF1, 0x1A, 0xB0, 0xF0, 0x8A, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x8B, 0x29, 0x51, 0x79,
            0x8A, 0x24, 0x70, 0x59, 0x8B, 0x20, 0x58, 0x71, 0x8A, 0x44, 0x69, 0x38, 0x8B, 0x39, 0x40, 0x68,
            0x8A, 0x64, 0x48, 0x31, 0x8B, 0x30, 0x49, 0x60, 0xA5, 0x88, 0x20, 0x09, 0x71, 0x58, 0x44, 0x68,
            # Bank 6
            0x11, 0x39, 0x64, 0x49, 0x30, 0x19, 0xF1, 0xAC, 0x00, 0x2C, 0x54, 0x7C, 0xF0, 0x8C, 0xA8, 0x04,
            0x28, 0x50, 0x78, 0xF1, 0x88, 0x97, 0x26, 0xA8, 0x59, 0x98, 0xAC, 0x8C, 0x02, 0x26, 0x46, 0x66,
            0xF0, 0x89, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31,
            0xA9, 0x88, 0x09, 0x20, 0x59, 0x70, 0xAB, 0x11, 0x38, 0x40, 0x69, 0xA8, 0x19, 0x31, 0x48, 0x60,
            0x8C, 0xA8, 0x3C, 0x41, 0x5C, 0x20, 0x7C, 0x00, 0xF1, 0x87, 0x98, 0x19, 0x86, 0xA8, 0x6E, 0x76,
            0x7E, 0xA9, 0x99, 0x88, 0x2D, 0x55, 0x7D, 0x9E, 0xB9, 0xA3, 0x8A, 0x22, 0x8A, 0x6E, 0x8A, 0x56,
            0x8A, 0x5E, 0x9F, 0xB1, 0x83, 0x06, 0x26, 0x46, 0x66, 0x0E, 0x2E, 0x4E, 0x6E, 0x9D, 0xB8, 0xAD,
            0x00, 0x2C, 0x54, 0x7C, 0xF2, 0xB1, 0x8C, 0xB4, 0x99, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0x81, 0x91,
            0xAC, 0x38, 0xAD, 0x3A, 0xB5, 0x83, 0x91, 0xAC, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9, 0x48, 0xD8,
            0x6D, 0xD9, 0x68, 0xD8, 0x8C, 0x9D, 0xAE, 0x29, 0xD9, 0x04, 0xAE, 0xD8, 0x51, 0xD9, 0x04, 0xAE,
            0xD8, 0x79, 0xD9, 0x04, 0xD8, 0x81, 0xF3, 0x9D, 0xAD, 0x00, 0x8D, 0xAE, 0x19, 0x81, 0xAD, 0xD9,
            0x01, 0xD8, 0xF2, 0xAE, 0xDA, 0x26, 0xD8, 0x8E, 0x91, 0x29, 0x83, 0xA7, 0xD9, 0xAD, 0xAD, 0xAD,
            0xAD, 0xF3, 0x2A, 0xD8, 0xD8, 0xF1, 0xB0, 0xAC, 0x89, 0x91, 0x3E, 0x5E, 0x76, 0xF3, 0xAC, 0x2E,
            0x2E, 0xF1, 0xB1, 0x8C, 0x5A, 0x9C, 0xAC, 0x2C, 0x28, 0x28, 0x28, 0x9C, 0xAC, 0x30, 0x18, 0xA8,
            0x98, 0x81, 0x28, 0x34, 0x3C, 0x97, 0x24, 0xA7, 0x28, 0x34, 0x3C, 0x9C, 0x24, 0xF2, 0xB0, 0x89,
            0xAC, 0x91, 0x2C, 0x4C, 0x6C, 0x8A, 0x9B, 0x2D, 0xD9, 0xD8, 0xD8, 0x51, 0xD9, 0xD8, 0xD8, 0x79,
            # Bank 7
            0xD9, 0xD8, 0xD8, 0xF1, 0x9E, 0x88, 0xA3, 0x31, 0xDA, 0xD8, 0xD8, 0x91, 0x2D, 0xD9, 0x28, 0xD8,
            0x4D, 0xD9, 0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x83, 0x93, 0x35, 0x3D, 0x80, 0x25, 0xDA,
            0xD8, 0xD8, 0x85, 0x69, 0xDA, 0xD8, 0xD8, 0xB4, 0x93, 0x81, 0xA3, 0x28, 0x34, 0x3C, 0xF3, 0xAB,
            0x8B, 0xF8, 0xA3, 0x91, 0xB6, 0x09, 0xB4, 0xD9, 0xAB, 0xDE, 0xFA, 0xB0, 0x87, 0x9C, 0xB9, 0xA3,
            0xDD, 0xF1, 0x20, 0x28, 0x30, 0x38, 0x9A, 0xF1, 0x28, 0x30, 0x38, 0x9D, 0xF1, 0xA3, 0xA3, 0xA3,
            0xA3, 0xF2, 0xA3, 0xB4, 0x90, 0x80, 0xF2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3,
            0xA3, 0xB2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xB0, 0x87, 0xB5, 0x99, 0xF1, 0x28, 0x30, 0x38,
            0x98, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x97, 0xA3, 0xA3, 0xA3, 0xA3, 0xF3, 0x9B, 0xA3, 0x30, 0xDC,
            0xB9, 0xA7, 0xF1, 0x26, 0x26, 0x26, 0xFE, 0xD8, 0xFF
            ]
        
        self.first_run_flag = False
        self.new_data_available = False
        self.calibration_values = {
                                    "ac_x" : 0,
                                    "ac_y" : 0,
                                    "ac_z" : 0.
                                    }
        
        time.sleep_ms(50) # Making sure the MPU6050 has had enough time to boot up before you start sending commands
        
        # Wake up MPU6050
        self.module.writeto_mem(IMUADDRESS, self.registers["pwr_mgmnt"], bytes([0x80]))
        time.sleep_ms(50)
        self.module.writeto_mem(IMUADDRESS, self.registers["pwr_mgmnt"], bytes([0x00]))
        self.module.writeto_mem(IMUADDRESS, self.registers["pwr_mgmnt_2"], bytes([0x00]))
        
        # Setting required accelerometer/gyro scalars (2g for the accelerometer, 2000deg/s for gyro)
        self.module.writeto_mem(IMUADDRESS, self.registers["accel_config"], bytearray([0x00]))
        self.module.writeto_mem(IMUADDRESS, self.registers["gyro_config"], bytearray([0x18]))
        
        self.data = bytearray(28)
    
    def _log(self, string):
        print(string)
                
    def dmpsetup(self, int_pin):
        no_banks = 8
        
        # Setting DLPF to 42Hz (gyros) and 44Hz (accel), and sample rate to 400Hz
        self.module.writeto_mem(IMUADDRESS, self.registers["config"], bytearray([0x03]))
        self.module.writeto_mem(IMUADDRESS, self.registers["smplrt_div"], bytearray([0x04]))
        self._log("Sample rates and filters set up")
        
        # Setting up FIFO
        self.module.writeto_mem(IMUADDRESS, self.registers["fifo_en"], bytearray([0x00]))
        self.module.writeto_mem(IMUADDRESS, self.registers["user_ctrl"], bytearray([0x04]))
        self._log("FIFO enabled")
        
        # Setting up DMP interrupts
        self.module.writeto_mem(IMUADDRESS, self.registers["int_enable"], bytearray([0x00]))
        self.module.writeto_mem(IMUADDRESS, self.registers["int_enable"], bytearray([0x02]))
        self.module.writeto_mem(IMUADDRESS, self.registers["int_pin_config"], bytearray([0x90]))
        
        self._log("Interrupts enabled")
        
        # Writes the DMP firmware to the DMP in 16-byte blocks
        for bank in range(no_banks):
            self._log("Loading firmware bank {}".format(bank))
            
            byte_offset = 0
            
            # Setting firmware bank being written into memory
            self.module.writeto_mem(IMUADDRESS, self.registers["dmp_ctrl_1"], bytearray([bank]))

            if bank == 7:
                limit = 137
            else:
                limit = 256
            
            while byte_offset < limit:
                    firmware_byte = self.dmp_firmware_v20[bank*256+byte_offset]
                    
                    # Writing the firmware byte
                    self.module.writeto_mem(IMUADDRESS, self.registers["dmp_ctrl_2"], bytearray([byte_offset]))
                    self.module.writeto_mem(IMUADDRESS, self.registers["dmp_ctrl_3"], bytearray([firmware_byte]))
                    
                    # Setting the byte we want to read back, and then reading it back to check
                    self.module.writeto_mem(IMUADDRESS, self.registers["dmp_ctrl_2"], bytearray([byte_offset]))
                    byte = self.module.readfrom_mem(IMUADDRESS, self.registers["dmp_ctrl_3"], 1)[0]
                    
                    if byte == firmware_byte:
                        byte_offset += 1
            
            time.sleep(0.01)
        
        # Writing firmware start byte
        self.module.writeto_mem(IMUADDRESS, self.registers["dmp_firmware_start_1"], bytearray([0x03]))
        self._log("Firmware started")
        
        # Enable FIFO and DMP, reset FIFO and DMP
        self.module.writeto_mem(IMUADDRESS, self.registers["user_ctrl"], bytearray([0xCC]))
        self._log("DMP and FIFO actived and reset")

        if self.module.readfrom_mem(IMUADDRESS, self.registers["WHO_AM_I"], 1)[0] != 0x68:
            raise I2CConnectionError
        
        # Activate pin-driven interrupts
        self.pin_interrupt = Pin(int_pin, Pin.IN)
        self.pin_interrupt.irq(trigger=Pin.IRQ_FALLING, handler=self._updatedata)
        self._log("Pin-driven interrupts activated")
    
    def calibrate(self, length):
        d_ax = d_ay = d_az = counter = ready = 0
        end_time = time.time()+length
        
        self._log("Calibrating")
        # Coarse calibration offsets
        while time.time() < end_time:
            self._newdata()
            
            d_ax += self.decode_acceL_data(self.data[16:18])
            d_ay += self.decode_acceL_data(self.data[18:20])
            d_az += self.decode_acceL_data(self.data[20:22]) - 9.81
            
            counter += 1
        
            time.sleep(0.1)
        
        d_ax /= counter
        d_ay /= counter
        d_az /= counter
        self._log("Coarse calibration complete")
        
        tolerance = 0.005
        divisor = 8
        
        # Fine tuning the calibration offsets        
        while ready != 3:
            self._newdata()
            
            ax = self.decode_acceL_data(self.data[16:18]) + d_ax
            ay = self.decode_acceL_data(self.data[18:20]) + d_ay
            az = self.decode_acceL_data(self.data[20:22]) + d_az
            
            ready = 0
            
            if abs(ax) > tolerance:
                d_ax -= ax/divisor
            else:
                ready += 1
            if abs(ay) > tolerance:
                d_ay -= ay/divisor
            else:
                ready += 1
            if abs(az-9.81) > tolerance:
                d_az -= (az-9.81)/divisor
            else:
                ready += 1
                                
            time.sleep(0.01)
        
        self._log("Fine calibration complete")
        
        self.calibration_values["ac_x"] = d_ax
        self.calibration_values["ac_y"] = d_ay
        self.calibration_values["ac_z"] = d_az
    
    @micropython.native
    def _newdata(self):
        if self.new_data_available:
            if self.module.readfrom_mem(IMUADDRESS, 0x3A, 1)[0] & 0x10: # Checking to see if FIFO overflow flag is set to 1
                register = self.module.readfrom_mem(IMUADDRESS, 0x6A, 1)[0]
                self.module.writeto_mem(IMUADDRESS, 0x6A, bytes([register|0x04])) # Resetting FIFO buffer without touching anything else
                time.sleep_ms(5) # Letting a new packet enter the FIFO: 200Hz update rate -> 5ms per data frame
            
            # Finding how many bytes there are in the FIFO
            count_h = self.module.readfrom_mem(IMUADDRESS, 0x72, 1)[0]
            count_l = self.module.readfrom_mem(IMUADDRESS, 0x73, 1)[0]
            count = (count_h << 8) | count_l
            
            # Gets data from FIFO buffer (quaternion + accelerometer)
            data_frame = self.module.readfrom_mem(IMUADDRESS, FIFO_REG, count)
            
            if len(data_frame) < 28:
                return None
            
            self.data = data_frame
            
            self.new_data_available = False
    
    @micropython.native
    def _updatedata(self, pin=None):
        self.new_data_available = True
        
    @micropython.native
    def _decode_acceL_data(self, data):
        data_point = struct.unpack(">h", data)[0]

        data_point /= 16384
        data_point *= 9.81
        
        return data_point

    @micropython.native
    def _decode_quat_data(self, data):
        data_point = struct.unpack(">l", data)[0]
            
        data_point /= 1073741824
        
        return data_point

    @micropython.native
    def quat_to_euler(self, q):
        qw, qx, qy, qz = q
        # Converts quaternion values from DMP to euler angles that are human-readable via NASA standard sequence (Z-Y-X)
        yaw = atan2(2*(qw*qz + qx*qy), 1-2*(qy*qy+qz*qz))
        roll = atan2(2*(qw*qx + qy*qz), 1-2*(qx*qx + qy*qy))
        
        arg = 2*(qw*qy - qx*qz)
        
        if arg > 1:
            arg = 1
        elif arg < -1:
            arg = -1
        
        pitch = asin(arg)
        
        # 57.2957795 approx. 180/pi
        pitch *= 57.2957795
        roll *= 57.2957795
        yaw *= 57.2957795
        
        return pitch, roll, yaw
    
    @micropython.native
    def _body_frame_acceleration(self, ax, ay, az, qw, qx, qy, qz):
        # Rotate gravity vector into body's reference frame to subract gravity from accelerometer readings. Gives acceleration in body's reference frame
         gx = 19.62*(qx*qz + qw*qy)
         gy = 19.62*(qy*qz - qw*qx)
         gz = 9.81*(qw*qw - qx*qx - qy*qy + qz*qz)
         
         ax -= gx
         ay -= gy
         az -= gz
         
         return ax, ay, az
    
    @micropython.native
    def _world_frame_acceleration(self, ax, ay, az, qw, qx, qy, qz):
        # Rotate acceleration vector into world reference frame, then subtract gravity from it. Gives acceleration in world reference frame
        w_ax = (qw*qw + qx*qx - qy*qy - qz*qz)*ax + 2*(qx*qy - qw*qz)*ay + 2*(qx*qz + qw*qy)*az
        w_ay = 2*(qx*qy + qw*qz)*ax + (qw*qw - qx*qx + qy*qy - qz*qz)*ay + 2*(qy*qz - qw*qx)*az
        w_az = 2*(qx*qz - qw*qy)*ax + 2*(qy*qz + qw*qx)*ay + (qw*qw - qx*qx - qy*qy + qz*qz)*az
        
        return w_ax, w_ay, w_az
    
    @micropython.native
    def _fastround(self, number): # Quick way of rounding POSITIVE floats to 2.d.p
        number *=100
        rounded = int(number+0.5)
        
        return rounded/100
    
    @micropython.native
    def imutrack(self):
        # Initialise self.start_time only on the first execution of this method
        if not self.first_run_flag:
            self.start_time = time.time_ns()
            self.first_run_flag = True
        
        self._newdata()
        
        # Extracting quaternion and acceleration data from the data frame and converting it to ints
        data = self.data
        
        qw = self.decode_quat_data(data[0:4])
        qx = self.decode_quat_data(data[4:8])
        qy = self.decode_quat_data(data[8:12])
        qz = self.decode_quat_data(data[12:16])
        
        ax = self._decode_accel_data(data[16:18]) + self.calibration_values["ac_x"]
        ay = self._decode_accel_data(data[18:20]) + self.calibration_values["ac_y"]
        az = self._decode_accel_data(data[20:22]) + self.calibration_values["ac_z"]
        
        ax = self._fastround(ax)
        ay = self._fastround(ay)
        az = self._fastround(az)
        
        # Normalize quaternion
        norm = (qw*qw + qx*qx + qy*qy + qz*qz)**0.5
        
        if norm == 0:
            qw, qx, qy, qz = 1, 0, 0, 0
        else:
            qw /= norm
            qx /= norm
            qy /= norm
            qz /= norm
        
            orientation = self.quat_to_euler(qw, qx, qy, qz)
        
        local_accel = self._body_frame_acceleration(ax, ay, az, qw, qx, qy, qz)
        world_accel = self._world_frame_acceleration(local_accel[0], local_accel[1], local_accel[2], qw, qx, qy, qz)
        
        return [qw, qx, qy, qz], local_accel, world_accel
        

if __name__ == "__main__":
    module = MPU6050(46, 3)
    module.dmpsetup(1)
    module.calibrate(10)
    
    while True:
        print(module.imutrack())