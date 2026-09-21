#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

struct MPUreadings{
  int8_t x;
  int8_t y;
  int8_t z;
};

class MPU6050{
  private: 
    uint8_t address;
    int16_t aOffX, aOffY, aOffZ;
    int16_t gOffX, gOffY, gOffZ;
  public:
    MPU6050(uint8_t address, int16_t gOffX, int16_t gOffY, int16_t gOffZ, int16_t aOffX, int16_t aOffY, int16_t aOffZ);
    void wakeSensor();
    MPUreadings readGyro();
    MPUreadings readAccel();
};

 
#endif