#include <SPI.h>
#include <mcp2515.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include "mpu6050.h"
#include "pot.h"

// ==========================================
// 1. GLOBAL SETTINGS & DEFINITIONS
// ==========================================
#define FRONT // comment this line out if compiling for the rear

const uint8_t CS_CAN = PIN_PB2;
const uint8_t POT_L = PIN_A7;
const uint8_t POT_R = PIN_PC0;
const uint8_t AMBIENT_NTC = PIN_PC3;

// NTC Array
const int8_t adc_to_temp[1024] PROGMEM = {-55, -55, -55, -55, -55, -55, -55, -55, -55, -55, -55, -55, -54, -53, -52, -51, -50, -49, -49, -48, -47, -46, -46, -45, -44, -44, -43, -43, -42, -42, -41, -41, -40, -40, -39, -39, -38, -38, -37, -37, -36, -36, -36, -35, -35, -35, -34, -34, -33, -33, -33, -32, -32, -32, -31, -31, -31, -30, -30, -30, -30, -29, -29, -29, -28, -28, -28, -28, -27, -27, -27, -27, -26, -26, -26, -26, -25, -25, -25, -25, -24, -24, -24, -24, -23, -23, -23, -23, -23, -22, -22, -22, -22, -21, -21, -21, -21, -21, -20, -20, -20, -20, -20, -19, -19, -19, -19, -19, -18, -18, -18, -18, -18, -18, -17, -17, -17, -17, -17, -16, -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, -15, -14, -14, -14, -14, -14, -14, -14, -13, -13, -13, -13, -13, -13, -12, -12, -12, -12, -12, -12, -12, -11, -11, -11, -11, -11, -11, -11, -10, -10, -10, -10, -10, -10, -10, -9, -9, -9, -9, -9, -9, -9, -8, -8, -8, -8, -8, -8, -8, -8, -7, -7, -7, -7, -7, -7, -7, -6, -6, -6, -6, -6, -6, -6, -6, -5, -5, -5, -5, -5, -5, -5, -5, -4, -4, -4, -4, -4, -4, -4, -4, -4, -3, -3, -3, -3, -3, -3, -3, -3, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 33, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 40, 40, 40, 40, 40, 40, 40, 40, 40, 41, 41, 41, 41, 41, 41, 41, 41, 41, 42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 43, 43, 43, 43, 43, 43, 43, 43, 43, 44, 44, 44, 44, 44, 44, 44, 44, 45, 45, 45, 45, 45, 45, 45, 45, 45, 46, 46, 46, 46, 46, 46, 46, 46, 47, 47, 47, 47, 47, 47, 47, 47, 47, 48, 48, 48, 48, 48, 48, 48, 48, 49, 49, 49, 49, 49, 49, 49, 50, 50, 50, 50, 50, 50, 50, 50, 51, 51, 51, 51, 51, 51, 51, 51, 52, 52, 52, 52, 52, 52, 52, 53, 53, 53, 53, 53, 53, 53, 53, 54, 54, 54, 54, 54, 54, 54, 55, 55, 55, 55, 55, 55, 56, 56, 56, 56, 56, 56, 56, 57, 57, 57, 57, 57, 57, 57, 58, 58, 58, 58, 58, 58, 59, 59, 59, 59, 59, 59, 60, 60, 60, 60, 60, 60, 61, 61, 61, 61, 61, 61, 62, 62, 62, 62, 62, 62, 63, 63, 63, 63, 63, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 65, 66, 66, 66, 66, 66, 67, 67, 67, 67, 67, 68, 68, 68, 68, 68, 69, 69, 69, 69, 70, 70, 70, 70, 70, 71, 71, 71, 71, 72, 72, 72, 72, 73, 73, 73, 73, 74, 74, 74, 74, 75, 75, 75, 75, 76, 76, 76, 76, 77, 77, 77, 77, 78, 78, 78, 79, 79, 79, 79, 80, 80, 80, 81, 81, 81, 82, 82, 82, 83, 83, 83, 84, 84, 84, 85, 85, 85, 86, 86, 86, 87, 87, 88, 88, 88, 89, 89, 89, 90, 90, 91, 91, 92, 92, 92, 93, 93, 94, 94, 95, 95, 96, 96, 97, 97, 98, 98, 99, 99, 100, 101, 101, 102, 102, 103, 104, 104, 105, 106, 106, 107, 108, 109, 109, 110, 111, 112, 113, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125, 125};
// ---------------------------
// FRONT / REAR OBJECTS
// ---------------------------
#ifdef FRONT
const uint8_t STEER = PIN_PC2;
Pot FLpot(POT_L, 35, 34, 2317);
Pot FRpot(POT_R, 70, 79, 2364);
MPU6050 mpu1(0x68, -24, -131, -124, -135, 35, 17243);
MPU6050 mpu4(0x69, -451, -29, -184, 765, -304, 17324);
struct can_frame can700;
struct can_frame can701;
struct can_frame can702;
#else
const uint8_t PUMP_FLOW = PIN_PD3;
const uint8_t PUMP_NTC = PIN_PC1;
Pot RLpot(POT_L, 50, 51, 2322);
Pot RRpot(POT_R, 185, 204, 2319);
MPU6050 mpu2(0x68, 225, 196, -306, -281, 383, 17277);
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

// ==========================================
// 2. TIMING & SYSTEM VARIABLES
// ==========================================
MCP2515 mcp2515(CS_CAN);
unsigned long last100ms = 0;
unsigned long last1000ms = 0;

// ==========================================
// 3. MAIN SETUP & LOOP
// ==========================================
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