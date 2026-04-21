#pragma once

#include <Arduino.h>

// Input pins
constexpr int INPUT2 = PIN_PB0; //general purpose
constexpr int AC_ISO = PIN_PC0; //Accumulator batteryside voltage analog input, voltage input 600V become 4.7244V
constexpr int TS_ISO = PIN_PC1; // Accumulator car side voltage analog input, voltage input 600V become 4.7244V, precharge higher than 95% in under 5 seconds otherwise fail
constexpr int HALL_LO = PIN_PC2; // Hall effect sensor low range analog input +-75A to 5V
constexpr int HALL_HI = PIN_PC3; // Hall effect sensor high range analog input +-500A to 5V
constexpr int LVB_5V = PIN_PC4; // low voltage battery voltage unused
constexpr int TS_ACT = PIN_PD6; // Tractive system active signal, active low
constexpr int SDC = PIN_PD7; // shutdown circuit signal, active high

// Output pins
constexpr int AMS_ERR = PIN_PD0; // AMS error signal, active low. over/undervoltage, overcurrent
constexpr int AIR_NEG = PIN_PD1; // Accumulator isolation relay negative side control, active low
constexpr int AIR_POS = PIN_PD2;    // Accumulator isolation relay positive side control, active low
constexpr int PRE = PIN_PD5; // Precharge relay control, active low