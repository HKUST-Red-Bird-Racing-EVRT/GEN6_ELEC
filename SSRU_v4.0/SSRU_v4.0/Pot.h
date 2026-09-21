#ifndef POT_H
#define POT_H
#include <Arduino.h>

class Pot{
  private:
    uint8_t pin;
    uint8_t num, den;
    uint16_t offset;
  public:
    Pot(uint8_t pin, uint8_t num, uint8_t den, uint16_t offset):
      pin(pin), num(num), den(den), offset(offset){}
    uint16_t readPot(){
      uint16_t rawPot = analogRead(pin);
      return ((uint32_t)rawPot * num) / den + offset;
    };
};
#endif