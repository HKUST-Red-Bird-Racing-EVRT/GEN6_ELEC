#include <Wire.h>
#include <stdint.h> 
#include "mpu6050.h" 

enum MPU6050_Registers : uint8_t
{ 
    PWR_MGMT_1  = 0x6B, // power management 1 (global)
    GYRO_XOUT_H = 0x43, // x-axis gyroscope - high byte
    ACCEL_XOUT_H = 0x3B, // x-axis accelerometer - high byte
};

MPU6050::MPU6050(uint8_t address, int16_t gOffX, int16_t gOffY, int16_t gOffZ, int16_t aOffX, int16_t aOffY, int16_t aOffZ):
  address(address), gOffX(gOffX), gOffY(gOffY), gOffZ(gOffZ), aOffX(aOffX), aOffY(aOffY), aOffZ(aOffZ){};

void MPU6050::wakeSensor(){
    Wire.beginTransmission(address);
    Wire.write(PWR_MGMT_1); // global power management register
    Wire.write(0x00); // wakes sensor up
    Wire.endTransmission(true);
};

// reads gyroscope data
MPUreadings MPU6050::readGyro(){
    MPUreadings gyro;
    Wire.beginTransmission(address); 
    Wire.write(GYRO_XOUT_H); // used as starting address to read all other values
    Wire.endTransmission(false);
    Wire.requestFrom(address, 6, true); // requests 6 bytes - 3 high values and 3 low values from given address

    auto scale = [](int16_t raw) -> int8_t {
            const int16_t LIMIT = 1638;     // ±12.5°/s at 131 LSB/°/s
            const uint16_t SPAN  = 3276;    // 1638 × 2
            const uint16_t TO    = 254;     // scale to 0..254

            if (raw <= -LIMIT || raw >= LIMIT) {
                return -128;                // out-of-range error
            }

            uint16_t offset = raw + LIMIT;           // → 0..3276
            uint16_t scaled = (offset * TO) / SPAN;  // → 0..254

            return scaled - 127;                     // → -127..0..+127
        };

    int16_t rawGyroX = (Wire.read() << 8); rawGyroX |= Wire.read();
    int16_t rawGyroY = (Wire.read() << 8); rawGyroY |= Wire.read();
    int16_t rawGyroZ = (Wire.read() << 8); rawGyroZ |= Wire.read();

    gyro.x = scale(rawGyroX - gOffX);
    gyro.y = scale(rawGyroY - gOffY);
    gyro.z = scale(rawGyroZ - gOffZ);

    return gyro;
 }; //-127 to 127 = -12.5 to 12.5

// reads accelerometer data
MPUreadings MPU6050::readAccel(){
    MPUreadings accel;
    Wire.beginTransmission(address);
    Wire.write(ACCEL_XOUT_H); // used as starting address to read all other values
    Wire.endTransmission(false);
    Wire.requestFrom(address, 6, true); // requests 6 bytes - 3 high values and 3 low values from given address


    int16_t rawAccelX = (Wire.read() << 8); rawAccelX |= Wire.read();
    int16_t rawAccelY = (Wire.read() << 8); rawAccelY |= Wire.read();
    int16_t rawAccelZ = (Wire.read() << 8); rawAccelZ |= Wire.read();

    accel.x = (rawAccelX - aOffX) >> 8;
    accel.y = (rawAccelY - aOffY) >> 8;
    accel.z = (rawAccelZ - aOffZ) >> 8;

    return accel;
}; //-128 to 127 = -2g to 2g

