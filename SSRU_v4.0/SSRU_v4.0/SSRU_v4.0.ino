#define FRONT 
#include <SPI.h>
#include <mcp2515.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include "mpu6050.h"
#include "pot.h"
#include "config.h"

#ifdef FRONT
struct can_frame can700;
struct can_frame can701;
struct can_frame can702;
#else
struct can_frame can710;
struct can_frame can711;

volatile int16_t pulseCount = 0;
void pulseCounter(){
  pulseCount++;
}
void flowScale(uint8_t &flowHigh, uint8_t &flowLow){
  noInterrupts();
  int16_t pulsesHz = pulseCount;
  pulseCount = 0;
  interrupts();
  uint16_t mLPerSec = ((uint32_t)pulsesHz*100)/66;
  flowHigh = mLPerSec >> 8;
  flowLow = mLPerSec & 0xFF;
}
#endif 

MCP2515 mcp2515(CS_CAN);
unsigned long last100ms = 0;
unsigned long last1000ms = 0;

void setup() {
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_20MHZ);
  mcp2515.setNormalMode();
  Wire.begin();
  
  #ifdef FRONT
  mpu1.wakeSensor();
  mpu4.wakeSensor();
    
  can700.can_id = 0x700;
  can700.can_dlc = 8;
    
  can701.can_id = 0x701;
  can701.can_dlc = 8;
    
  can702.can_id = 0x702;
  can702.can_dlc = 3;
  
  #else
  mpu2.wakeSensor();
    
  can710.can_id = 0x710;
  can710.can_dlc = 8;
    
  can711.can_id = 0x711;
  can711.can_dlc = 6; // Fixed typo here (was can711.can_id = 6;)
    
  pinMode(PUMP_FLOW, INPUT);
  attachInterrupt(digitalPinToInterrupt(PUMP_FLOW), pulseCounter, FALLING);
  #endif
}

void loop() {
  unsigned long currentMillis = millis();

  // ===========
  // 100ms LOOP
  // ===========
  if (currentMillis - last100ms >= 100) {
    last100ms = currentMillis;

    #ifdef FRONT
    // --- ID 0x700 ---
    uint16_t potL = FLpot.readPot();
    MPUreadings mpu1A = mpu1.readAccel();
    MPUreadings mpu1G = mpu1.readGyro();
    
    can700.data[0] = potL >> 8;
    can700.data[1] = potL & 0xFF;
    can700.data[2] = mpu1A.x;
    can700.data[3] = mpu1A.y;
    can700.data[4] = mpu1A.z;
    can700.data[5] = mpu1G.x;
    can700.data[6] = mpu1G.y;
    can700.data[7] = mpu1G.z;
    mcp2515.sendMessage(&can700);

    // --- ID 0x701 ---
    uint16_t potR = FRpot.readPot();
    MPUreadings mpu4A = mpu4.readAccel();
    MPUreadings mpu4G = mpu4.readGyro();
    
    can701.data[0] = potR >> 8;
    can701.data[1] = potR & 0xFF;
    can701.data[2] = mpu4A.x;
    can701.data[3] = mpu4A.y;
    can701.data[4] = mpu4A.z;
    can701.data[5] = mpu4G.x;
    can701.data[6] = mpu4G.y;
    can701.data[7] = mpu4G.z;
    mcp2515.sendMessage(&can701);

    // --- ID 0x702 ---
    uint16_t steer = analogRead(STEER);
    can702.data[0] = steer >> 8;
    can702.data[1] = steer & 0xFF;
    mcp2515.sendMessage(&can702);


    #else
    // --- ID 0x710 ---
    uint16_t potL = RLpot.readPot();
    MPUreadings mpu2A = mpu2.readAccel();
    MPUreadings mpu2G = mpu2.readGyro();
    
    can710.data[0] = potL >> 8;
    can710.data[1] = potL & 0xFF;
    can710.data[2] = mpu2A.x;
    can710.data[3] = mpu2A.y;
    can710.data[4] = mpu2A.z;
    can710.data[5] = mpu2G.x;
    can710.data[6] = mpu2G.y;
    can710.data[7] = mpu2G.z;
    mcp2515.sendMessage(&can710);

    // --- ID 0x711 ---
    uint16_t potR = RRpot.readPot();
    can711.data[0] = potR >> 8;
    can711.data[1] = potR & 0xFF;
    mcp2515.sendMessage(&can711);

    #endif
  }

  // ============
  // 1000ms LOOP
  // ============
  if (currentMillis - last1000ms >= 1000) {
    last1000ms = currentMillis;

    #ifdef FRONT
    uint16_t ambRaw = analogRead(AMBIENT_NTC);
    can702.data[2] = (int8_t)pgm_read_byte(&adc_to_temp[ambRaw]);
    
    #else
    uint16_t ambRaw = analogRead(AMBIENT_NTC);
    uint16_t pumpRaw = analogRead(PUMP_NTC);
    
    can711.data[2] = (int8_t)pgm_read_byte(&adc_to_temp[ambRaw]);
    can711.data[3] = (int8_t)pgm_read_byte(&adc_to_temp[pumpRaw]);

    uint8_t fHigh, fLow;
    flowScale(fHigh, fLow);
    can711.data[4] = fHigh;
    can711.data[5] = fLow;
    #endif
  }
}